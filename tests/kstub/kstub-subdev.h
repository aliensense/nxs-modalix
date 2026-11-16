/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * kstub-subdev.h - the V4L2 sub-device interfaces nxs-csi2.c uses, for a user-space test
 *
 * Copyright (C) 2026 Aliensense
 *
 * The <media/v4l2-*.h> headers the sub-device includes resolve to this one.
 * The endpoint, the device properties and the registrations are the test's:
 * the endpoint is the one kstub_endpoint describes, the properties are the
 * ones the test set with kstub_property_set(), and a registration records
 * the sub-device for i2c_get_clientdata(). A control event subscription
 * marks the control of its id on the sub-device's handler.
 */

#ifndef NXS_KSTUB_SUBDEV_H
#define NXS_KSTUB_SUBDEV_H

#include "kstub.h"

/* ---- the endpoint and the device properties --------------------------- */

#define V4L2_MBUS_CSI2_DPHY	5

struct fwnode_handle {
	int unused;
};

struct v4l2_fwnode_endpoint {
	int bus_type;
	struct {
		struct {
			unsigned short num_data_lanes;
		} mipi_csi2;
	} bus;
	u64 *link_frequencies;
	unsigned int nr_of_link_frequencies;
};

struct v4l2_fwnode_device_properties {
	int orientation;
	int rotation;
};

/* The endpoint the test's node carries: its lanes and its link frequencies. */
extern unsigned short kstub_endpoint_lanes;
extern u64 kstub_endpoint_freqs[4];
extern unsigned int kstub_endpoint_nr_freqs;

struct fwnode_handle *dev_fwnode(struct device *dev);
struct fwnode_handle *fwnode_graph_get_next_endpoint(struct fwnode_handle *fwnode,
						     struct fwnode_handle *prev);
void fwnode_handle_put(struct fwnode_handle *fwnode);
int v4l2_fwnode_endpoint_alloc_parse(struct fwnode_handle *fwnode,
				     struct v4l2_fwnode_endpoint *ep);
void v4l2_fwnode_endpoint_free(struct v4l2_fwnode_endpoint *ep);
int v4l2_fwnode_device_parse(struct device *dev, struct v4l2_fwnode_device_properties *props);
int v4l2_ctrl_new_fwnode_properties(struct v4l2_ctrl_handler *hdl,
				    const struct v4l2_ctrl_ops *ops,
				    const struct v4l2_fwnode_device_properties *props);

bool device_property_read_bool(struct device *dev, const char *name);
bool device_property_present(struct device *dev, const char *name);
int device_property_count_u32(struct device *dev, const char *name);
int device_property_read_u32_array(struct device *dev, const char *name, u32 *val, size_t n);

/* The node carries @name: a flag for a NULL @cells, else @n u32 cells, which the test keeps. */
void kstub_property_set(const char *name, const u32 *cells, size_t n);
/* The node carries no property. */
void kstub_properties_clear(void);

/* ---- formats and selections ------------------------------------------- */

#define V4L2_FIELD_NONE			1
#define V4L2_COLORSPACE_RAW		11
#define V4L2_YCBCR_ENC_DEFAULT		0
#define V4L2_QUANTIZATION_FULL_RANGE	1
#define V4L2_XFER_FUNC_NONE		5

#define V4L2_SEL_TGT_CROP		0x0000
#define V4L2_SEL_TGT_CROP_DEFAULT	0x0001
#define V4L2_SEL_TGT_CROP_BOUNDS	0x0002
#define V4L2_SEL_TGT_NATIVE_SIZE	0x0003

#define V4L2_SUBDEV_FORMAT_TRY		0
#define V4L2_SUBDEV_FORMAT_ACTIVE	1

struct v4l2_rect {
	s32 left;
	s32 top;
	u32 width;
	u32 height;
};

struct v4l2_mbus_framefmt {
	u32 width;
	u32 height;
	u32 code;
	u32 field;
	u32 colorspace;
	u16 ycbcr_enc;
	u16 quantization;
	u16 xfer_func;
};

struct v4l2_subdev_format {
	u32 which;
	u32 pad;
	struct v4l2_mbus_framefmt format;
};

struct v4l2_subdev_mbus_code_enum {
	u32 pad;
	u32 index;
	u32 code;
};

struct v4l2_subdev_frame_size_enum {
	u32 index;
	u32 pad;
	u32 code;
	u32 min_width;
	u32 max_width;
	u32 min_height;
	u32 max_height;
};

struct v4l2_subdev_selection {
	u32 which;
	u32 pad;
	u32 target;
	struct v4l2_rect r;
};

/* ---- the sub-device --------------------------------------------------- */

#define MEDIA_PAD_FL_SOURCE		(1U << 1)
#define MEDIA_ENT_F_CAM_SENSOR		0x00020001
#define V4L2_SUBDEV_FL_HAS_DEVNODE	(1U << 2)
#define V4L2_SUBDEV_FL_HAS_EVENTS	(1U << 3)

struct media_pad {
	unsigned long flags;
};

struct media_entity {
	u32 function;
};

struct v4l2_subdev_state {
	struct v4l2_mbus_framefmt format;
};

struct v4l2_subdev;

struct v4l2_fh {
	int unused;
};

struct v4l2_event_subscription {
	u32 type;
	u32 id;
	u32 flags;
};

struct v4l2_subdev_video_ops {
	int (*s_stream)(struct v4l2_subdev *sd, int enable);
	int (*pre_streamon)(struct v4l2_subdev *sd, u32 flags);
};

struct v4l2_subdev_pad_ops {
	int (*enum_mbus_code)(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			      struct v4l2_subdev_mbus_code_enum *code);
	int (*enum_frame_size)(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			       struct v4l2_subdev_frame_size_enum *fse);
	int (*get_fmt)(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *format);
	int (*set_fmt)(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
		       struct v4l2_subdev_format *format);
	int (*get_selection)(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			     struct v4l2_subdev_selection *sel);
};

struct v4l2_subdev_ops {
	const struct v4l2_subdev_video_ops *video;
	const struct v4l2_subdev_pad_ops *pad;
};

struct v4l2_subdev_internal_ops {
	int (*init_state)(struct v4l2_subdev *sd, struct v4l2_subdev_state *state);
};

struct v4l2_subdev {
	const struct v4l2_subdev_ops *ops;
	const struct v4l2_subdev_internal_ops *internal_ops;
	struct v4l2_ctrl_handler *ctrl_handler;
	u32 flags;
	struct media_entity entity;
	struct mutex *state_lock;
};

void v4l2_i2c_subdev_init(struct v4l2_subdev *sd, struct i2c_client *client,
			  const struct v4l2_subdev_ops *ops);
int media_entity_pads_init(struct media_entity *entity, u16 num_pads, struct media_pad *pads);
void media_entity_cleanup(struct media_entity *entity);
int v4l2_subdev_init_finalize(struct v4l2_subdev *sd);
void v4l2_subdev_cleanup(struct v4l2_subdev *sd);
int v4l2_async_register_subdev_sensor(struct v4l2_subdev *sd);
void v4l2_async_unregister_subdev(struct v4l2_subdev *sd);
/* No active state: the test drives the pad ops with a state of its own. */
struct v4l2_subdev_state *v4l2_subdev_get_locked_active_state(struct v4l2_subdev *sd);
struct v4l2_mbus_framefmt *v4l2_subdev_state_get_format(struct v4l2_subdev_state *state,
							unsigned int pad);
/*
 * A V4L2_EVENT_CTRL subscription to a control on the sub-device's handler,
 * EINVAL for none. V4L2_EVENT_SUB_FL_SEND_INITIAL sends the initial event.
 */
int v4l2_ctrl_subdev_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				     struct v4l2_event_subscription *sub);
int v4l2_event_subdev_unsubscribe(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				  struct v4l2_event_subscription *sub);

/* ---- the driver ------------------------------------------------------- */

struct of_device_id {
	char compatible[128];
};

struct i2c_driver {
	struct {
		const char *name;
		const struct of_device_id *of_match_table;
	} driver;
	int (*probe)(struct i2c_client *client);
	void (*remove)(struct i2c_client *client);
};

#define MODULE_DEVICE_TABLE(type, name)	extern int kstub_device_table
#define module_i2c_driver(drv)		struct i2c_driver *kstub_i2c_driver = &(drv)

#endif /* NXS_KSTUB_SUBDEV_H */
