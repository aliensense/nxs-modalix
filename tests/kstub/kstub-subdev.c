// SPDX-License-Identifier: GPL-2.0-only
/*
 * kstub-subdev.c - the V4L2 sub-device interfaces of kstub-subdev.h, for a user-space test
 *
 * Copyright (C) 2026 Aliensense
 */

#include "kstub-subdev.h"

unsigned short kstub_endpoint_lanes = 4;
u64 kstub_endpoint_freqs[4] = { 1188000000 };
unsigned int kstub_endpoint_nr_freqs = 1;

static struct fwnode_handle kstub_fwnode;

struct fwnode_handle *dev_fwnode(struct device *dev)
{
	(void)dev;

	return &kstub_fwnode;
}

struct fwnode_handle *fwnode_graph_get_next_endpoint(struct fwnode_handle *fwnode,
						     struct fwnode_handle *prev)
{
	return prev ? NULL : fwnode;
}

void fwnode_handle_put(struct fwnode_handle *fwnode)
{
	(void)fwnode;
}

int v4l2_fwnode_endpoint_alloc_parse(struct fwnode_handle *fwnode,
				     struct v4l2_fwnode_endpoint *ep)
{
	(void)fwnode;
	ep->bus.mipi_csi2.num_data_lanes = kstub_endpoint_lanes;
	ep->nr_of_link_frequencies = kstub_endpoint_nr_freqs;
	ep->link_frequencies = kstub_endpoint_freqs;

	return 0;
}

void v4l2_fwnode_endpoint_free(struct v4l2_fwnode_endpoint *ep)
{
	ep->link_frequencies = NULL;
}

int v4l2_fwnode_device_parse(struct device *dev, struct v4l2_fwnode_device_properties *props)
{
	(void)dev;
	memset(props, 0, sizeof(*props));

	return 0;
}

int v4l2_ctrl_new_fwnode_properties(struct v4l2_ctrl_handler *hdl,
				    const struct v4l2_ctrl_ops *ops,
				    const struct v4l2_fwnode_device_properties *props)
{
	(void)ops;
	(void)props;

	return hdl->error;
}

/* The properties the test's node carries. */
struct kstub_property {
	const char *name;
	const u32 *cells;
	size_t n;
};

static struct kstub_property kstub_properties[8];
static unsigned int kstub_property_count;

void kstub_property_set(const char *name, const u32 *cells, size_t n)
{
	struct kstub_property *p;

	if (kstub_property_count >= ARRAY_SIZE(kstub_properties)) {
		fprintf(stderr, "kstub: more than %zu properties\n", ARRAY_SIZE(kstub_properties));
		abort();
	}
	p = &kstub_properties[kstub_property_count++];
	p->name = name;
	p->cells = cells;
	p->n = cells ? n : 0;
}

void kstub_properties_clear(void)
{
	kstub_property_count = 0;
}

static const struct kstub_property *kstub_property(const char *name)
{
	unsigned int i;

	for (i = 0; i < kstub_property_count; i++)
		if (!strcmp(kstub_properties[i].name, name))
			return &kstub_properties[i];

	return NULL;
}

bool device_property_read_bool(struct device *dev, const char *name)
{
	(void)dev;

	return kstub_property(name) != NULL;
}

bool device_property_present(struct device *dev, const char *name)
{
	(void)dev;

	return kstub_property(name) != NULL;
}

int device_property_count_u32(struct device *dev, const char *name)
{
	const struct kstub_property *p = kstub_property(name);

	(void)dev;

	return p ? (int)p->n : -EINVAL;
}

/* As the kernel's: EOVERFLOW for more cells than the property holds. */
int device_property_read_u32_array(struct device *dev, const char *name, u32 *val, size_t n)
{
	const struct kstub_property *p = kstub_property(name);

	(void)dev;
	if (!p)
		return -EINVAL;
	if (n > p->n)
		return -EOVERFLOW;
	memcpy(val, p->cells, n * sizeof(*val));

	return 0;
}

void v4l2_i2c_subdev_init(struct v4l2_subdev *sd, struct i2c_client *client,
			  const struct v4l2_subdev_ops *ops)
{
	sd->ops = ops;
	client->clientdata = sd;
}

int media_entity_pads_init(struct media_entity *entity, u16 num_pads, struct media_pad *pads)
{
	(void)entity;
	(void)num_pads;
	(void)pads;

	return 0;
}

void media_entity_cleanup(struct media_entity *entity)
{
	(void)entity;
}

int v4l2_subdev_init_finalize(struct v4l2_subdev *sd)
{
	(void)sd;

	return 0;
}

void v4l2_subdev_cleanup(struct v4l2_subdev *sd)
{
	(void)sd;
}

int v4l2_async_register_subdev_sensor(struct v4l2_subdev *sd)
{
	(void)sd;

	return 0;
}

void v4l2_async_unregister_subdev(struct v4l2_subdev *sd)
{
	(void)sd;
}

struct v4l2_subdev_state *v4l2_subdev_get_locked_active_state(struct v4l2_subdev *sd)
{
	(void)sd;

	return NULL;
}

struct v4l2_mbus_framefmt *v4l2_subdev_state_get_format(struct v4l2_subdev_state *state,
							unsigned int pad)
{
	(void)pad;

	return &state->format;
}

static struct v4l2_ctrl *kstub_subscribed_ctrl(struct v4l2_subdev *sd,
					       const struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_CTRL || !sd->ctrl_handler)
		return NULL;

	return kstub_ctrl_find(sd->ctrl_handler, sub->id);
}

int v4l2_ctrl_subdev_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				     struct v4l2_event_subscription *sub)
{
	struct v4l2_ctrl *ctrl = kstub_subscribed_ctrl(sd, sub);

	(void)fh;
	if (!ctrl)
		return -EINVAL;
	ctrl->subscribed = true;
	if (sub->flags & V4L2_EVENT_SUB_FL_SEND_INITIAL)
		kstub_send_initial_event(ctrl);

	return 0;
}

int v4l2_event_subdev_unsubscribe(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				  struct v4l2_event_subscription *sub)
{
	struct v4l2_ctrl *ctrl = kstub_subscribed_ctrl(sd, sub);

	(void)fh;
	if (ctrl)
		ctrl->subscribed = false;

	return 0;
}
