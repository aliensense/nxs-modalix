/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * adapter.h - nxs-csi2.c, the V4L2 sub-device, as a test drives it
 *
 * Copyright (C) 2026 Aliensense
 *
 * test-adapter.c builds the sub-device driver with its statics in reach and
 * offers these calls on a client it probed. Each runs as the framework
 * would: a pad op or a control set under the sub-device's lock, a stream op
 * outside it.
 */

#ifndef NXS_TEST_ADAPTER_H
#define NXS_TEST_ADAPTER_H

#include <media/v4l2-subdev.h>

#include "nxs-csi2-core.h"

int adapter_probe(struct i2c_client *client);
void adapter_remove(struct i2c_client *client);
/* The core the sub-device holds. */
struct nxs_csi2_core *adapter_core(struct i2c_client *client);
/* The index of the line row and the line in effect in ns. */
int adapter_line_row(struct i2c_client *client);
u32 adapter_line_ns(struct i2c_client *client);
/* The sub-device's lock: the handler's and the state's. */
struct mutex *adapter_lock(struct i2c_client *client);
/* The control of @id on the sub-device's handler, NULL for none. */
struct v4l2_ctrl *adapter_ctrl(struct i2c_client *client, u32 id);
/* The controls on the sub-device's handler. */
unsigned int adapter_ctrl_count(struct i2c_client *client);
/* A read of the control of @id from user space, as the framework runs it (kstub_ctrl_get()). */
int adapter_get_ctrl(struct i2c_client *client, u32 id, s32 *val);
/* A set of the control of @id from user space, as the framework runs it (kstub_ctrl_set()). */
int adapter_set_ctrl(struct i2c_client *client, u32 id, s32 val);
/* One set of the @n controls of @ids, one cluster's, to @vals (kstub_ctrls_set()). */
int adapter_set_ctrls(struct i2c_client *client, const u32 *ids, const s32 *vals, unsigned int n);
/* One try of the @n controls of @ids, one cluster's, at @vals (kstub_ctrls_try()). */
int adapter_try_ctrls(struct i2c_client *client, const u32 *ids, const s32 *vals, unsigned int n);
/*
 * A client subscribes to the V4L2_EVENT_CTRL of the control of @id, or
 * unsubscribes, as the sub-device's node does: ENOIOCTLCMD for a
 * sub-device without the events flag or a control handler.
 */
int adapter_subscribe(struct i2c_client *client, u32 id, bool on);
/* A subscription as adapter_subscribe() with V4L2_EVENT_SUB_FL_SEND_INITIAL. */
int adapter_subscribe_initial(struct i2c_client *client, u32 id);
/* A format ask: the door read again where its description changed. @fmt may be NULL. */
int adapter_get_fmt(struct i2c_client *client, struct v4l2_mbus_framefmt *fmt);
/* An active format set of @fmt, which takes the format the sub-device reports. */
int adapter_set_fmt(struct i2c_client *client, struct v4l2_mbus_framefmt *fmt);
/* A try format set of @fmt, which takes the format the sub-device reports. */
int adapter_try_fmt(struct i2c_client *client, struct v4l2_mbus_framefmt *fmt);
/* A stream start or stop, outside the lock as the framework calls it: the op takes the lock. */
int adapter_s_stream(struct i2c_client *client, bool on);
bool adapter_streaming(struct i2c_client *client);
/* HFLIP or VFLIP, NULL for none. */
struct v4l2_ctrl *adapter_flip(struct i2c_client *client, bool vertical);
/* A set of the flips cluster to @h and @v: the members whose value changes, through the master. */
int adapter_set_flips(struct i2c_client *client, bool h, bool v);

#endif /* NXS_TEST_ADAPTER_H */
