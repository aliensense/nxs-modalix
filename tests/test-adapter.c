// SPDX-License-Identifier: GPL-2.0-only
/*
 * test-adapter.c - nxs-csi2.c built for user space, its statics in reach
 *
 * Copyright (C) 2026 Aliensense
 */

#include "nxs-csi2.c"

#include "adapter.h"

static struct nxs_csi2 *adapter_of(struct i2c_client *client)
{
	return to_nxs_csi2(i2c_get_clientdata(client));
}

int adapter_probe(struct i2c_client *client)
{
	return nxs_csi2_probe(client);
}

void adapter_remove(struct i2c_client *client)
{
	nxs_csi2_remove(client);
}

struct nxs_csi2_core *adapter_core(struct i2c_client *client)
{
	return &adapter_of(client)->core;
}

int adapter_line_row(struct i2c_client *client)
{
	return adapter_of(client)->line_row;
}

u32 adapter_line_ns(struct i2c_client *client)
{
	return adapter_of(client)->line_ns;
}

struct mutex *adapter_lock(struct i2c_client *client)
{
	return &adapter_of(client)->lock;
}

struct v4l2_ctrl *adapter_ctrl(struct i2c_client *client, u32 id)
{
	return kstub_ctrl_find(&adapter_of(client)->ctrls, id);
}

unsigned int adapter_ctrl_count(struct i2c_client *client)
{
	return kstub_ctrl_count(&adapter_of(client)->ctrls);
}

int adapter_get_ctrl(struct i2c_client *client, u32 id, s32 *val)
{
	struct v4l2_ctrl *ctrl = adapter_ctrl(client, id);

	return ctrl ? kstub_ctrl_get(ctrl, val) : -EINVAL;
}

int adapter_set_ctrl(struct i2c_client *client, u32 id, s32 val)
{
	struct v4l2_ctrl *ctrl = adapter_ctrl(client, id);

	return ctrl ? kstub_ctrl_set(ctrl, val) : -EINVAL;
}

/* The controls of @ids, one cluster's, into @ctrls: -EINVAL for an id with none. */
static int adapter_ctrls(struct i2c_client *client, const u32 *ids, unsigned int n,
			 struct v4l2_ctrl **ctrls)
{
	unsigned int i;

	if (n > KSTUB_CLUSTER_MAX)
		return -EINVAL;
	for (i = 0; i < n; i++) {
		ctrls[i] = adapter_ctrl(client, ids[i]);
		if (!ctrls[i])
			return -EINVAL;
	}

	return 0;
}

int adapter_set_ctrls(struct i2c_client *client, const u32 *ids, const s32 *vals, unsigned int n)
{
	struct v4l2_ctrl *ctrls[KSTUB_CLUSTER_MAX];

	return adapter_ctrls(client, ids, n, ctrls) ?: kstub_ctrls_set(ctrls, vals, n);
}

int adapter_try_ctrls(struct i2c_client *client, const u32 *ids, const s32 *vals, unsigned int n)
{
	struct v4l2_ctrl *ctrls[KSTUB_CLUSTER_MAX];

	return adapter_ctrls(client, ids, n, ctrls) ?: kstub_ctrls_try(ctrls, vals, n);
}

/*
 * As the kernel's subdev_do_ioctl() runs VIDIOC_SUBSCRIBE_EVENT and
 * VIDIOC_UNSUBSCRIBE_EVENT for a sub-device without core operations: the
 * control framework's own, on a node with the events flag and, to
 * subscribe, a control handler.
 */
static int adapter_subscribe_flags(struct i2c_client *client, u32 id, bool on, u32 flags)
{
	struct v4l2_subdev *sd = &adapter_of(client)->sd;
	struct v4l2_event_subscription sub = { .type = V4L2_EVENT_CTRL, .id = id, .flags = flags };
	struct v4l2_fh fh = { };

	if (!(sd->flags & V4L2_SUBDEV_FL_HAS_EVENTS))
		return -ENOIOCTLCMD;
	if (!on)
		return v4l2_event_subdev_unsubscribe(sd, &fh, &sub);

	return sd->ctrl_handler ? v4l2_ctrl_subdev_subscribe_event(sd, &fh, &sub) : -ENOIOCTLCMD;
}

int adapter_subscribe(struct i2c_client *client, u32 id, bool on)
{
	return adapter_subscribe_flags(client, id, on, 0);
}

int adapter_subscribe_initial(struct i2c_client *client, u32 id)
{
	return adapter_subscribe_flags(client, id, true, V4L2_EVENT_SUB_FL_SEND_INITIAL);
}

int adapter_get_fmt(struct i2c_client *client, struct v4l2_mbus_framefmt *fmt)
{
	struct nxs_csi2 *priv = adapter_of(client);
	struct v4l2_subdev_format format = { };
	struct v4l2_subdev_state state = { };
	int err;

	mutex_lock(priv->sd.state_lock);
	err = nxs_csi2_get_fmt(&priv->sd, &state, &format);
	mutex_unlock(priv->sd.state_lock);
	if (fmt)
		*fmt = format.format;

	return err;
}

static int adapter_set_fmt_which(struct i2c_client *client, u32 which,
				  struct v4l2_mbus_framefmt *fmt)
{
	struct nxs_csi2 *priv = adapter_of(client);
	struct v4l2_subdev_format format = { .which = which, .format = *fmt };
	struct v4l2_subdev_state state = { };
	int err;

	mutex_lock(priv->sd.state_lock);
	err = nxs_csi2_set_fmt(&priv->sd, &state, &format);
	mutex_unlock(priv->sd.state_lock);
	*fmt = format.format;

	return err;
}

int adapter_set_fmt(struct i2c_client *client, struct v4l2_mbus_framefmt *fmt)
{
	return adapter_set_fmt_which(client, V4L2_SUBDEV_FORMAT_ACTIVE, fmt);
}

int adapter_try_fmt(struct i2c_client *client, struct v4l2_mbus_framefmt *fmt)
{
	return adapter_set_fmt_which(client, V4L2_SUBDEV_FORMAT_TRY, fmt);
}

int adapter_s_stream(struct i2c_client *client, bool on)
{
	struct nxs_csi2 *priv = adapter_of(client);

	return nxs_csi2_s_stream(&priv->sd, on);
}

bool adapter_streaming(struct i2c_client *client)
{
	return adapter_of(client)->streaming;
}

struct v4l2_ctrl *adapter_flip(struct i2c_client *client, bool vertical)
{
	struct nxs_csi2 *priv = adapter_of(client);

	return vertical ? priv->vflip : priv->hflip;
}

int adapter_set_flips(struct i2c_client *client, bool h, bool v)
{
	struct nxs_csi2 *priv = adapter_of(client);
	struct v4l2_ctrl *master = priv->hflip ? priv->hflip : priv->vflip;
	unsigned int i;
	int err;

	if (priv->hflip) {
		priv->hflip->val = h;
		priv->hflip->has_changed = priv->hflip->cur.val != h;
	}
	if (priv->vflip) {
		priv->vflip->val = v;
		priv->vflip->has_changed = priv->vflip->cur.val != v;
	}
	mutex_lock(&priv->lock);
	err = master->ops->s_ctrl(master);
	mutex_unlock(&priv->lock);
	for (i = 0; !err && i < master->ncontrols; i++)
		master->cluster[i]->cur.val = master->cluster[i]->val;

	return err;
}
