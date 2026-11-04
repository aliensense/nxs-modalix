// SPDX-License-Identifier: GPL-2.0-only
/*
 * nxs-hub-route.c - the routing node of an NXS Hub's CSI-2 output
 *
 * Copyright (C) 2026 Aliensense
 *
 * An NXS Hub's deserializer puts each link's camera on a virtual channel of
 * one CSI-2 output. The nxs host tool programs the deserializer over I2C, so
 * no kernel driver writes it. This node is the output's place in the media
 * graph: a sink pad per link, bound to the link's camera door (nxs-csi2), and
 * one source pad toward the receiver. It writes no register.
 *
 * Its frame descriptor names each link's stream with the virtual channel the
 * device tree gives the link and the data type of the camera door's format,
 * and the stream's id is that virtual channel. A receiver that routes by
 * stream starts a link through the node, and the node starts the link's
 * camera door. A receiver that takes virtual channel k on its pixel
 * interface k finds every link in the descriptor, and each of its streams
 * starts its own link. A receiver that starts one stream for all its
 * interfaces takes aliensense,links-start-together: the output's first
 * stream then starts every routed link, and its last stream stops them.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include <media/mipi-csi2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

/* Sink pads 0 to 3, one per link; the source pad is the Hub's output. */
#define NXS_ROUTE_LINKS 4
#define NXS_ROUTE_SOURCE NXS_ROUTE_LINKS
#define NXS_ROUTE_PADS (NXS_ROUTE_LINKS + 1)
/* CSI-2 virtual channels 0 to 3 on a D-PHY link. */
#define NXS_ROUTE_VCS 4

struct nxs_route_link {
	bool present;
	u32 vc;
	/* The camera door bound to the link's sink pad, and its source pad. */
	struct v4l2_subdev *door;
	unsigned int door_pad;
};

struct nxs_route {
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pads[NXS_ROUTE_PADS];
	struct v4l2_async_notifier notifier;
	struct v4l2_ctrl_handler ctrls;
	struct nxs_route_link links[NXS_ROUTE_LINKS];
	/* The output's endpoint: its data lanes and its one link frequency. */
	u8 lanes;
	s64 link_freq;
	/* The receiver starts one stream for every link (aliensense,links-start-together). */
	bool together;
	/* The output's streams the receiver started; the active state's lock guards them. */
	u64 running;
};

struct nxs_route_asd {
	struct v4l2_async_connection base;
	unsigned int link;
};

static inline struct nxs_route *to_nxs_route(struct v4l2_subdev *sd)
{
	return container_of(sd, struct nxs_route, sd);
}

static inline struct nxs_route_asd *to_nxs_route_asd(struct v4l2_async_connection *asc)
{
	return container_of(asc, struct nxs_route_asd, base);
}

/* A stream's format before the link's camera door names its own. */
static const struct v4l2_mbus_framefmt nxs_route_default_fmt = {
	.width = 1920,
	.height = 1080,
	.code = MEDIA_BUS_FMT_SRGGB10_1X10,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_RAW,
	.ycbcr_enc = V4L2_YCBCR_ENC_601,
	.quantization = V4L2_QUANTIZATION_FULL_RANGE,
	.xfer_func = V4L2_XFER_FUNC_NONE,
};

/* The CSI-2 data type a camera door's format rides; 0 for a code it has none for. */
static u8 nxs_route_data_type(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_Y8_1X8:
		return MIPI_CSI2_DT_RAW8;
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_Y10_1X10:
		return MIPI_CSI2_DT_RAW10;
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_Y12_1X12:
		return MIPI_CSI2_DT_RAW12;
	case MEDIA_BUS_FMT_SBGGR14_1X14:
	case MEDIA_BUS_FMT_SGBRG14_1X14:
	case MEDIA_BUS_FMT_SGRBG14_1X14:
	case MEDIA_BUS_FMT_SRGGB14_1X14:
		return MIPI_CSI2_DT_RAW14;
	case MEDIA_BUS_FMT_SBGGR16_1X16:
	case MEDIA_BUS_FMT_SGBRG16_1X16:
	case MEDIA_BUS_FMT_SGRBG16_1X16:
	case MEDIA_BUS_FMT_SRGGB16_1X16:
		return MIPI_CSI2_DT_RAW16;
	default:
		return 0;
	}
}

/* ---- the routes ---------------------------------------------------------- */

/*
 * The routes the device tree allows: link i's stream 0 to the output's stream
 * of the link's virtual channel. A table may leave a link's route inactive.
 */
static int nxs_route_check_routing(struct nxs_route *priv,
				   const struct v4l2_subdev_krouting *routing)
{
	unsigned int i;

	for (i = 0; i < routing->num_routes; i++) {
		const struct v4l2_subdev_route *route = &routing->routes[i];

		if (route->sink_pad >= NXS_ROUTE_LINKS || !priv->links[route->sink_pad].present ||
		    route->sink_stream != 0 || route->source_pad != NXS_ROUTE_SOURCE ||
		    route->source_stream != priv->links[route->sink_pad].vc) {
			dev_dbg(priv->dev, "route %u/%u -> %u/%u is not a link's\n",
				route->sink_pad, route->sink_stream, route->source_pad,
				route->source_stream);
			return -EINVAL;
		}
	}

	return 0;
}

static int nxs_route_apply_routing(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				   struct v4l2_subdev_krouting *routing)
{
	struct nxs_route *priv = to_nxs_route(sd);
	int err;

	err = nxs_route_check_routing(priv, routing);
	if (err)
		return err;
	err = v4l2_subdev_routing_validate(sd, routing, V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
	if (err)
		return err;

	return v4l2_subdev_set_routing_with_fmt(sd, state, routing, &nxs_route_default_fmt);
}

static int nxs_route_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	struct nxs_route *priv = to_nxs_route(sd);
	struct v4l2_subdev_route routes[NXS_ROUTE_LINKS] = {};
	struct v4l2_subdev_krouting routing = { .routes = routes };
	unsigned int i;

	for (i = 0; i < NXS_ROUTE_LINKS; i++) {
		if (!priv->links[i].present)
			continue;
		routes[routing.num_routes++] = (struct v4l2_subdev_route) {
			.sink_pad = i,
			.sink_stream = 0,
			.source_pad = NXS_ROUTE_SOURCE,
			.source_stream = priv->links[i].vc,
			.flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE,
		};
	}

	return nxs_route_apply_routing(sd, state, &routing);
}

static int nxs_route_set_routing(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				 enum v4l2_subdev_format_whence which,
				 struct v4l2_subdev_krouting *routing)
{
	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	return nxs_route_apply_routing(sd, state, routing);
}

/* ---- the formats --------------------------------------------------------- */

/*
 * A link's format is its camera door's: an active set goes to the door first,
 * and the format the door took lands on the link's sink and on its stream of
 * the output. The output's formats follow the links.
 */
static int nxs_route_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			     struct v4l2_subdev_format *format)
{
	struct nxs_route *priv = to_nxs_route(sd);
	struct nxs_route_link *link;
	struct v4l2_mbus_framefmt *fmt;

	if (format->pad == NXS_ROUTE_SOURCE)
		return v4l2_subdev_get_fmt(sd, state, format);
	if (format->pad >= NXS_ROUTE_LINKS || format->stream != 0)
		return -EINVAL;
	link = &priv->links[format->pad];
	if (!link->present)
		return -EINVAL;
	if (format->which == V4L2_SUBDEV_FORMAT_ACTIVE && link->door) {
		struct v4l2_subdev_format door_fmt = {
			.which = V4L2_SUBDEV_FORMAT_ACTIVE,
			.pad = link->door_pad,
			.format = format->format,
		};
		int err;

		if (v4l2_subdev_is_streaming(sd))
			return -EBUSY;
		err = v4l2_subdev_call_state_active(link->door, pad, set_fmt, &door_fmt);
		if (err)
			return err;
		format->format = door_fmt.format;
	}
	fmt = v4l2_subdev_state_get_format(state, format->pad, format->stream);
	if (!fmt)
		return -EINVAL;
	*fmt = format->format;
	fmt = v4l2_subdev_state_get_opposite_stream_format(state, format->pad, format->stream);
	if (fmt)
		*fmt = format->format;

	return 0;
}

/* The camera door's active format onto its link's sink and its stream of the output. */
static void nxs_route_take_door_fmt(struct nxs_route *priv, unsigned int index)
{
	struct nxs_route_link *link = &priv->links[index];
	struct v4l2_subdev_format door_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = link->door_pad,
	};
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;

	if (v4l2_subdev_call_state_active(link->door, pad, get_fmt, &door_fmt))
		return;
	state = v4l2_subdev_lock_and_get_active_state(&priv->sd);
	fmt = v4l2_subdev_state_get_format(state, index, 0);
	if (fmt)
		*fmt = door_fmt.format;
	fmt = v4l2_subdev_state_get_opposite_stream_format(state, index, 0);
	if (fmt)
		*fmt = door_fmt.format;
	v4l2_subdev_unlock_state(state);
}

/* ---- the output's description ------------------------------------------------ */

static int nxs_route_get_frame_desc(struct v4l2_subdev *sd, unsigned int pad,
				    struct v4l2_mbus_frame_desc *fd)
{
	struct nxs_route *priv = to_nxs_route(sd);
	struct v4l2_subdev_state *state;
	struct v4l2_subdev_route *route;
	int err = 0;

	if (pad != NXS_ROUTE_SOURCE)
		return -EINVAL;

	memset(fd, 0, sizeof(*fd));
	fd->type = V4L2_MBUS_FRAME_DESC_TYPE_CSI2;
	state = v4l2_subdev_lock_and_get_active_state(sd);
	for_each_active_route(&state->routing, route) {
		struct v4l2_mbus_frame_desc_entry *entry;
		const struct v4l2_mbus_framefmt *fmt;

		if (fd->num_entries >= V4L2_FRAME_DESC_ENTRY_MAX) {
			err = -ENOSPC;
			break;
		}
		fmt = v4l2_subdev_state_get_format(state, route->sink_pad, route->sink_stream);
		if (!fmt) {
			err = -EINVAL;
			break;
		}
		entry = &fd->entry[fd->num_entries++];
		entry->stream = route->source_stream;
		entry->pixelcode = fmt->code;
		entry->bus.csi2.vc = priv->links[route->sink_pad].vc;
		entry->bus.csi2.dt = nxs_route_data_type(fmt->code);
	}
	v4l2_subdev_unlock_state(state);

	return err;
}

static int nxs_route_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				     struct v4l2_mbus_config *config)
{
	struct nxs_route *priv = to_nxs_route(sd);
	unsigned int i;

	if (pad != NXS_ROUTE_SOURCE)
		return -EINVAL;

	memset(config, 0, sizeof(*config));
	config->type = V4L2_MBUS_CSI2_DPHY;
	config->link_freq = priv->link_freq;
	config->bus.mipi_csi2.num_data_lanes = priv->lanes;
	for (i = 0; i < priv->lanes; i++)
		config->bus.mipi_csi2.data_lanes[i] = i + 1;

	return 0;
}

/* ---- the streams --------------------------------------------------------------- */

/* The output's streams the routing table carries. */
static u64 nxs_route_streams(struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_route *route;
	u64 streams = 0;

	for_each_active_route(&state->routing, route)
		streams |= BIT_ULL(route->source_stream);

	return streams;
}

/*
 * Start or stop the camera doors of the output's `streams`. A door keeps its
 * own stream state, and a start that fails part way stops the doors it
 * started.
 */
static int nxs_route_doors(struct nxs_route *priv, struct v4l2_subdev_state *state,
			   u64 streams, bool enable)
{
	struct v4l2_subdev_route *route;
	u64 started = 0;
	int err = 0;

	for_each_active_route(&state->routing, route) {
		struct nxs_route_link *link = &priv->links[route->sink_pad];
		int ret;

		if (!(streams & BIT_ULL(route->source_stream)) || !link->door)
			continue;
		ret = enable ? v4l2_subdev_enable_streams(link->door, link->door_pad, BIT_ULL(0))
			     : v4l2_subdev_disable_streams(link->door, link->door_pad, BIT_ULL(0));
		if (ret == -EALREADY)
			ret = 0;
		if (ret) {
			dev_err(priv->dev, "link %u: %s %s its stream: %d\n", route->sink_pad,
				link->door->name, enable ? "did not start" : "did not stop", ret);
			if (enable) {
				nxs_route_doors(priv, state, started, false);
				return ret;
			}
			err = err ?: ret;
			continue;
		}
		started |= BIT_ULL(route->source_stream);
	}

	return err;
}

static int nxs_route_enable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				    u32 pad, u64 streams_mask)
{
	struct nxs_route *priv = to_nxs_route(sd);
	/* The output's first stream starts every link. */
	u64 doors = !priv->together ? streams_mask : priv->running ? 0 : nxs_route_streams(state);
	int err;

	err = nxs_route_doors(priv, state, doors, true);
	if (!err)
		priv->running |= streams_mask;

	return err;
}

static int nxs_route_disable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				     u32 pad, u64 streams_mask)
{
	struct nxs_route *priv = to_nxs_route(sd);
	u64 running = priv->running & ~streams_mask;
	/* The output's last stream stops every link. */
	u64 doors = !priv->together ? streams_mask : running ? 0 : nxs_route_streams(state);
	int err;

	err = nxs_route_doors(priv, state, doors, false);
	if (!err)
		priv->running = running;

	return err;
}

static const struct v4l2_subdev_pad_ops nxs_route_pad_ops = {
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = nxs_route_set_fmt,
	.get_frame_desc = nxs_route_get_frame_desc,
	.get_mbus_config = nxs_route_get_mbus_config,
	.set_routing = nxs_route_set_routing,
	.enable_streams = nxs_route_enable_streams,
	.disable_streams = nxs_route_disable_streams,
};

static const struct v4l2_subdev_video_ops nxs_route_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_ops nxs_route_subdev_ops = {
	.video = &nxs_route_video_ops,
	.pad = &nxs_route_pad_ops,
};

static const struct v4l2_subdev_internal_ops nxs_route_internal_ops = {
	.init_state = nxs_route_init_state,
};

static const struct media_entity_operations nxs_route_entity_ops = {
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.link_validate = v4l2_subdev_link_validate,
	.has_pad_interdep = v4l2_subdev_has_pad_interdep,
};

/* ---- the camera doors ----------------------------------------------------------- */

static int nxs_route_notify_bound(struct v4l2_async_notifier *notifier, struct v4l2_subdev *door,
				  struct v4l2_async_connection *asc)
{
	struct nxs_route *priv = container_of(notifier, struct nxs_route, notifier);
	unsigned int index = to_nxs_route_asd(asc)->link;
	struct nxs_route_link *link = &priv->links[index];
	int pad, err;

	pad = media_entity_get_fwnode_pad(&door->entity, asc->match.fwnode, MEDIA_PAD_FL_SOURCE);
	if (pad < 0) {
		dev_err(priv->dev, "link %u: %s has no source pad for the endpoint\n", index,
			door->name);
		return pad;
	}
	err = media_create_pad_link(&door->entity, pad, &priv->sd.entity, index,
				    MEDIA_LNK_FL_ENABLED | MEDIA_LNK_FL_IMMUTABLE);
	if (err) {
		dev_err(priv->dev, "link %u: no media link to %s: %d\n", index, door->name, err);
		return err;
	}
	link->door = door;
	link->door_pad = pad;
	nxs_route_take_door_fmt(priv, index);
	dev_info(priv->dev, "link %u: %s on virtual channel %u\n", index, door->name, link->vc);

	return 0;
}

static void nxs_route_notify_unbind(struct v4l2_async_notifier *notifier, struct v4l2_subdev *door,
				    struct v4l2_async_connection *asc)
{
	struct nxs_route *priv = container_of(notifier, struct nxs_route, notifier);

	priv->links[to_nxs_route_asd(asc)->link].door = NULL;
}

static const struct v4l2_async_notifier_operations nxs_route_notify_ops = {
	.bound = nxs_route_notify_bound,
	.unbind = nxs_route_notify_unbind,
};

/* ---- the device tree ---------------------------------------------------------------- */

/* The output's endpoint: its data lanes and its one link frequency. */
static int nxs_route_parse_output(struct nxs_route *priv)
{
	struct v4l2_fwnode_endpoint ep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	struct fwnode_handle *endpoint;
	int err;

	endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(priv->dev), NXS_ROUTE_SOURCE, 0, 0);
	if (!endpoint)
		return dev_err_probe(priv->dev, -EINVAL, "port %u, the output, has no endpoint\n",
				     NXS_ROUTE_SOURCE);
	err = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (err)
		return dev_err_probe(priv->dev, err, "the output's endpoint\n");
	if (ep.nr_of_link_frequencies != 1) {
		err = dev_err_probe(priv->dev, -EINVAL,
				    "the output runs at one link frequency, %u named\n",
				    ep.nr_of_link_frequencies);
		goto free;
	}
	priv->lanes = ep.bus.mipi_csi2.num_data_lanes;
	priv->link_freq = ep.link_frequencies[0];
free:
	v4l2_fwnode_endpoint_free(&ep);

	return err;
}

/* Each link's sink port: its virtual channel, and its camera door to wait for. */
static int nxs_route_parse_links(struct nxs_route *priv)
{
	unsigned int i, links = 0;
	u32 taken = 0;
	int err;

	for (i = 0; i < NXS_ROUTE_LINKS; i++) {
		struct nxs_route_link *link = &priv->links[i];
		struct fwnode_handle *endpoint;
		struct nxs_route_asd *asd;

		endpoint = fwnode_graph_get_endpoint_by_id(dev_fwnode(priv->dev), i, 0, 0);
		if (!endpoint)
			continue;
		err = fwnode_property_read_u32(endpoint, "aliensense,virtual-channel", &link->vc);
		if (err || link->vc >= NXS_ROUTE_VCS || (taken & BIT(link->vc))) {
			fwnode_handle_put(endpoint);
			return dev_err_probe(priv->dev, -EINVAL,
					     "link %u: aliensense,virtual-channel names no free channel 0 to %u\n",
					     i, NXS_ROUTE_VCS - 1);
		}
		taken |= BIT(link->vc);
		asd = v4l2_async_nf_add_fwnode_remote(&priv->notifier, endpoint,
						      struct nxs_route_asd);
		fwnode_handle_put(endpoint);
		if (IS_ERR(asd))
			return dev_err_probe(priv->dev, PTR_ERR(asd), "link %u: its camera door\n",
					     i);
		asd->link = i;
		link->present = true;
		links++;
	}
	if (!links)
		return dev_err_probe(priv->dev, -EINVAL, "no link: ports 0 to %u carry none\n",
				     NXS_ROUTE_LINKS - 1);

	return 0;
}

static int nxs_route_init_controls(struct nxs_route *priv)
{
	struct v4l2_ctrl *ctrl;

	v4l2_ctrl_handler_init(&priv->ctrls, 1);
	ctrl = v4l2_ctrl_new_int_menu(&priv->ctrls, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      &priv->link_freq);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	if (priv->ctrls.error)
		return priv->ctrls.error;
	priv->sd.ctrl_handler = &priv->ctrls;

	return 0;
}

/* ---- probe and remove ------------------------------------------------------------------ */

static int nxs_route_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nxs_route *priv;
	unsigned int i;
	int err;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = dev;
	priv->together = device_property_read_bool(dev, "aliensense,links-start-together");
	platform_set_drvdata(pdev, priv);

	err = nxs_route_parse_output(priv);
	if (err)
		return err;

	v4l2_subdev_init(&priv->sd, &nxs_route_subdev_ops);
	priv->sd.internal_ops = &nxs_route_internal_ops;
	priv->sd.dev = dev;
	priv->sd.owner = THIS_MODULE;
	priv->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_STREAMS;
	priv->sd.entity.function = MEDIA_ENT_F_VID_MUX;
	priv->sd.entity.ops = &nxs_route_entity_ops;
	strscpy(priv->sd.name, dev_name(dev), sizeof(priv->sd.name));

	err = nxs_route_init_controls(priv);
	if (err)
		goto ctrls;

	for (i = 0; i < NXS_ROUTE_LINKS; i++)
		priv->pads[i].flags = MEDIA_PAD_FL_SINK;
	priv->pads[NXS_ROUTE_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	err = media_entity_pads_init(&priv->sd.entity, NXS_ROUTE_PADS, priv->pads);
	if (err)
		goto ctrls;

	v4l2_async_subdev_nf_init(&priv->notifier, &priv->sd);
	err = nxs_route_parse_links(priv);
	if (err)
		goto notifier;
	priv->notifier.ops = &nxs_route_notify_ops;

	err = v4l2_subdev_init_finalize(&priv->sd);
	if (err)
		goto notifier;
	err = v4l2_async_nf_register(&priv->notifier);
	if (err)
		goto state;
	err = v4l2_async_register_subdev(&priv->sd);
	if (err)
		goto unregister;
	dev_info(dev, "%u lanes at %lld Hz, register-free%s\n", priv->lanes, priv->link_freq,
		 priv->together ? ", the links start together" : "");

	return 0;

unregister:
	v4l2_async_nf_unregister(&priv->notifier);
state:
	v4l2_subdev_cleanup(&priv->sd);
notifier:
	v4l2_async_nf_cleanup(&priv->notifier);
	media_entity_cleanup(&priv->sd.entity);
ctrls:
	v4l2_ctrl_handler_free(&priv->ctrls);

	return err;
}

static void nxs_route_remove(struct platform_device *pdev)
{
	struct nxs_route *priv = platform_get_drvdata(pdev);

	v4l2_async_unregister_subdev(&priv->sd);
	v4l2_async_nf_unregister(&priv->notifier);
	v4l2_async_nf_cleanup(&priv->notifier);
	v4l2_subdev_cleanup(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->ctrls);
}

static const struct of_device_id nxs_route_of_match[] = {
	{ .compatible = "aliensense,hub-route" },
	{ }
};
MODULE_DEVICE_TABLE(of, nxs_route_of_match);

static struct platform_driver nxs_route_driver = {
	.driver = {
		.name = "nxs-hub-route",
		.of_match_table = nxs_route_of_match,
	},
	.probe = nxs_route_probe,
	.remove = nxs_route_remove,
};
module_platform_driver(nxs_route_driver);

MODULE_DESCRIPTION("Register-free routing node of an NXS Hub's CSI-2 output");
MODULE_AUTHOR("Aliensense");
MODULE_LICENSE("GPL");
