/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * kstub.h - the kernel interfaces nxs-csi2-core.c and nxs-csi2.c use, for a user-space test
 *
 * Copyright (C) 2026 Aliensense
 *
 * Every <linux/...> header and <media/v4l2-ctrls.h> resolve to this one, the
 * sub-device's other <media/...> headers to kstub-subdev.h. The types, the bit and bitmap helpers, the allocations and the
 * logging behave as the kernel's do. The I2C transfers, the bus lock, the
 * sleeps and the V4L2 controls are the test's: i2c_transfer() reaches the
 * fake door through the adapter, and the controls are plain records the
 * test drives the way the control framework does.
 */

#ifndef NXS_KSTUB_H
#define NXS_KSTUB_H

#include <errno.h>
#include <limits.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- types and limits ---------------------------------------------- */

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

#define U16_MAX		((u16)~0U)
#define U32_MAX		((u32)~0U)
#define S32_MAX		((s32)(U32_MAX >> 1))
#define S32_MIN		(-S32_MAX - 1)
#define S64_MAX		INT64_MAX

/* The kernel's numbers for the codes the host's errno.h may lack. */
#ifndef EREMOTEIO
#define EREMOTEIO	121
#endif
#ifndef ETIME
#define ETIME		62
#endif
#ifndef ENODATA
#define ENODATA		61
#endif
#ifndef EBADMSG
#define EBADMSG		74
#endif
#ifndef EPROTO
#define EPROTO		71
#endif
#define EPROBE_DEFER	517
#define ENOIOCTLCMD	515

/* ---- helpers ------------------------------------------------------- */

#define ARRAY_SIZE(a)		(sizeof(a) / sizeof((a)[0]))
#define container_of(ptr, type, member)	((type *)((char *)(ptr) - offsetof(type, member)))
#define NSEC_PER_SEC		1000000000L
#define round_down(x, y)	((x) & ~((__typeof__(x))((y) - 1)))
#define DIV_ROUND_CLOSEST_ULL(x, d)	(((unsigned long long)(x) + (d) / 2) / (d))
#define DIV64_U64_ROUND_UP(ll, d)	(((u64)(ll) + (d) - 1) / (d))
#define BIT_ULL(n)		(1ULL << (n))
#define GENMASK_ULL(h, l)	((~0ULL >> (63 - (h))) & (~0ULL << (l)))
#define DIV_ROUND_UP(n, d)	(((n) + (d) - 1) / (d))
#define min(a, b)		((a) < (b) ? (a) : (b))
#define max(a, b)		((a) > (b) ? (a) : (b))
#define min_t(t, a, b)		((t)(a) < (t)(b) ? (t)(a) : (t)(b))
#define max_t(t, a, b)		((t)(a) > (t)(b) ? (t)(a) : (t)(b))
#define clamp_t(t, v, lo, hi)	min_t(t, max_t(t, v, lo), hi)
#define __stringify_1(x)	#x
#define __stringify(x)		__stringify_1(x)
#ifndef static_assert
#define static_assert(expr, msg)	_Static_assert(expr, msg)
#endif
#define EXPORT_SYMBOL_GPL(sym)	extern int kstub_exported_##sym
#define MODULE_DESCRIPTION(s)	extern int kstub_module_description
#define MODULE_AUTHOR(s)	extern int kstub_module_author
#define MODULE_LICENSE(s)	extern int kstub_module_license
#define MODULE_VERSION(s)	const char *const kstub_module_version = s

extern const char *const kstub_module_version;

static inline u64 div_u64(u64 dividend, u32 divisor)
{
	return dividend / divisor;
}

static inline u64 div64_u64(u64 dividend, u64 divisor)
{
	return dividend / divisor;
}

static inline int fls64(u64 x)
{
	return x ? 64 - __builtin_clzll(x) : 0;
}

static inline unsigned long __ffs64(u64 x)
{
	return __builtin_ctzll(x);
}

static inline size_t strscpy(char *dst, const char *src, size_t n)
{
	size_t len = strnlen(src, n - 1);

	memcpy(dst, src, len);
	dst[len] = '\0';

	return len;
}

/* ---- bitmaps ------------------------------------------------------- */

#define BITS_PER_LONG		(8 * sizeof(long))
#define BITS_TO_LONGS(n)	DIV_ROUND_UP(n, BITS_PER_LONG)
#define DECLARE_BITMAP(name, bits)	unsigned long name[BITS_TO_LONGS(bits)]

static inline void __set_bit(unsigned int nr, unsigned long *map)
{
	map[nr / BITS_PER_LONG] |= 1UL << (nr % BITS_PER_LONG);
}

static inline void __clear_bit(unsigned int nr, unsigned long *map)
{
	map[nr / BITS_PER_LONG] &= ~(1UL << (nr % BITS_PER_LONG));
}

static inline bool test_bit(unsigned int nr, const unsigned long *map)
{
	return (map[nr / BITS_PER_LONG] >> (nr % BITS_PER_LONG)) & 1;
}

static inline unsigned int find_next_bit(const unsigned long *map, unsigned int size,
					 unsigned int start)
{
	for (; start < size; start++)
		if (test_bit(start, map))
			return start;

	return size;
}

static inline unsigned int find_next_zero_bit(const unsigned long *map, unsigned int size,
					      unsigned int start)
{
	for (; start < size; start++)
		if (!test_bit(start, map))
			return start;

	return size;
}

static inline unsigned int find_first_bit(const unsigned long *map, unsigned int size)
{
	return find_next_bit(map, size, 0);
}

#define for_each_set_bit(bit, map, size) \
	for ((bit) = find_first_bit(map, size); (bit) < (size); \
	     (bit) = find_next_bit(map, size, (bit) + 1))

static inline void bitmap_zero(unsigned long *map, unsigned int bits)
{
	memset(map, 0, BITS_TO_LONGS(bits) * sizeof(long));
}

static inline bool bitmap_empty(const unsigned long *map, unsigned int bits)
{
	return find_first_bit(map, bits) >= bits;
}

static inline void bitmap_clear(unsigned long *map, unsigned int start, unsigned int n)
{
	unsigned int i;

	for (i = start; i < start + n; i++)
		map[i / BITS_PER_LONG] &= ~(1UL << (i % BITS_PER_LONG));
}

static inline void bitmap_or(unsigned long *dst, const unsigned long *a, const unsigned long *b,
			     unsigned int bits)
{
	unsigned int i;

	for (i = 0; i < BITS_TO_LONGS(bits); i++)
		dst[i] = a[i] | b[i];
}

static inline void bitmap_andnot(unsigned long *dst, const unsigned long *a,
				 const unsigned long *b, unsigned int bits)
{
	unsigned int i;

	for (i = 0; i < BITS_TO_LONGS(bits); i++)
		dst[i] = a[i] & ~b[i];
}

static inline bool bitmap_intersects(const unsigned long *a, const unsigned long *b,
				     unsigned int bits)
{
	unsigned int i;

	for (i = 0; i < BITS_TO_LONGS(bits); i++)
		if (a[i] & b[i])
			return true;

	return false;
}

/* ---- devices and logging ------------------------------------------- */

struct device {
	const char *name;
};

/*
 * A mutex as the kernel's: not recursive. It knows its name, its holder and
 * its takes. A take while held, a release while free and a failed
 * lockdep_assert_held() print a line that names the mutex and abort the test.
 */
struct mutex {
	const char *name;
	/* The function that took it, NULL while free. */
	const char *holder;
	unsigned int takes;
};

void kstub_mutex_init(struct mutex *lock, const char *name);
void kstub_mutex_lock(struct mutex *lock, const char *func);
void kstub_mutex_unlock(struct mutex *lock, const char *func);
void kstub_lockdep_assert_held(const struct mutex *lock, const char *func);

#define mutex_init(lock)		kstub_mutex_init(lock, #lock)
#define mutex_lock(lock)		kstub_mutex_lock(lock, __func__)
#define mutex_unlock(lock)		kstub_mutex_unlock(lock, __func__)
#define lockdep_assert_held(lock)	kstub_lockdep_assert_held(lock, __func__)

static inline void mutex_destroy(struct mutex *lock)
{
	(void)lock;
}

void kstub_log(const struct device *dev, const char *level, const char *fmt, ...)
	__attribute__((format(printf, 3, 4)));
/* Forget the lines the core printed. */
void kstub_log_clear(void);
/* Whether a line the core printed since kstub_log_clear() holds @text. */
bool kstub_log_holds(const char *text);

#define dev_err(dev, ...)		kstub_log(dev, "err", __VA_ARGS__)
#define dev_warn(dev, ...)		kstub_log(dev, "warn", __VA_ARGS__)
#define dev_info(dev, ...)		kstub_log(dev, "info", __VA_ARGS__)
#define dev_dbg(dev, ...)		kstub_log(dev, "dbg", __VA_ARGS__)
#define dev_warn_ratelimited(dev, ...)	kstub_log(dev, "warn", __VA_ARGS__)
#define dev_info_ratelimited(dev, ...)	kstub_log(dev, "info", __VA_ARGS__)
#define dev_err_ratelimited(dev, ...)	kstub_log(dev, "err", __VA_ARGS__)
#define dev_err_probe(dev, err, ...)	(kstub_log(dev, "err", __VA_ARGS__), (err))

/* ---- allocations --------------------------------------------------- */

#define GFP_KERNEL	0

static inline void *kcalloc(size_t n, size_t size, int gfp)
{
	(void)gfp;

	return calloc(n, size);
}

static inline void kfree(const void *p)
{
	free((void *)p);
}

static inline void *kvmalloc(size_t size, int gfp)
{
	(void)gfp;

	return malloc(size);
}

/* kvcalloc() calls left to fail: the test's allocation error. */
extern unsigned int kstub_kvcalloc_failures;

static inline void *kvcalloc(size_t n, size_t size, int gfp)
{
	(void)gfp;
	if (kstub_kvcalloc_failures) {
		kstub_kvcalloc_failures--;
		return NULL;
	}

	return calloc(n, size);
}

static inline void kvfree(const void *p)
{
	free((void *)p);
}

void *devm_kcalloc(struct device *dev, size_t n, size_t size, int gfp);
void *devm_kzalloc(struct device *dev, size_t size, int gfp);
void *devm_kmemdup(struct device *dev, const void *src, size_t len, int gfp);
char *devm_kasprintf(struct device *dev, int gfp, const char *fmt, ...)
	__attribute__((format(printf, 3, 4)));
int devm_add_action_or_reset(struct device *dev, void (*action)(void *), void *data);
/* Runs the actions and frees the allocations of @dev, as an unbind does. */
void kstub_devm_release(struct device *dev);

/* ---- time ---------------------------------------------------------- */

void msleep(unsigned int ms);
void usleep_range(unsigned long min_us, unsigned long max_us);

/* ---- I2C ----------------------------------------------------------- */

#define I2C_M_RD		0x0001
#define I2C_FUNC_I2C		0x00000001
#define I2C_LOCK_SEGMENT	0x02
#define I2C_AQ_COMB		(1 << 0)

struct i2c_msg {
	u16 addr;
	u16 flags;
	u16 len;
	u8 *buf;
};

struct i2c_adapter_quirks {
	u64 flags;
	int max_num_msgs;
	u16 max_write_len;
	u16 max_read_len;
	u16 max_comb_1st_msg_len;
	u16 max_comb_2nd_msg_len;
};

struct i2c_adapter {
	const struct i2c_adapter_quirks *quirks;
	/* The test's device behind the adapter, and its transfer hook. */
	void *fake;
	int (*xfer)(void *fake, struct i2c_msg *msgs, int n, bool locked);
	/* Depth of the bus lock the caller holds. */
	int locked;
	/* Transfers that broke the lock rule: __i2c_transfer unlocked, i2c_transfer locked. */
	int lock_errors;
	/* Transfers the quirks refused, as the I2C core does: a read past max_read_len. */
	int quirk_errors;
};

struct i2c_client {
	u16 addr;
	struct i2c_adapter *adapter;
	struct device dev;
	/* What the driver registered against the client: the sub-device. */
	void *clientdata;
};

static inline bool i2c_check_functionality(struct i2c_adapter *adap, u32 func)
{
	(void)adap;

	return func == I2C_FUNC_I2C;
}

static inline void *i2c_get_clientdata(const struct i2c_client *client)
{
	return client->clientdata;
}

int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num);
int __i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num);
void i2c_lock_bus(struct i2c_adapter *adap, unsigned int flags);
void i2c_unlock_bus(struct i2c_adapter *adap, unsigned int flags);

/* ---- media bus codes ----------------------------------------------- */

#define MEDIA_BUS_FMT_Y8_1X8		0x2001
#define MEDIA_BUS_FMT_Y10_1X10		0x200a
#define MEDIA_BUS_FMT_Y12_1X12		0x2013
#define MEDIA_BUS_FMT_SBGGR8_1X8	0x3001
#define MEDIA_BUS_FMT_SGBRG8_1X8	0x3013
#define MEDIA_BUS_FMT_SGRBG8_1X8	0x3002
#define MEDIA_BUS_FMT_SRGGB8_1X8	0x3014
#define MEDIA_BUS_FMT_SBGGR10_1X10	0x3007
#define MEDIA_BUS_FMT_SGBRG10_1X10	0x300e
#define MEDIA_BUS_FMT_SGRBG10_1X10	0x300a
#define MEDIA_BUS_FMT_SRGGB10_1X10	0x300f
#define MEDIA_BUS_FMT_SBGGR12_1X12	0x3008
#define MEDIA_BUS_FMT_SGBRG12_1X12	0x3010
#define MEDIA_BUS_FMT_SGRBG12_1X12	0x3011
#define MEDIA_BUS_FMT_SRGGB12_1X12	0x3012

/* ---- V4L2 controls ------------------------------------------------- */

#define V4L2_CID_USER_BASE		0x00980900
#define V4L2_CID_EXPOSURE		0x00980911
#define V4L2_CID_HFLIP			0x00980914
#define V4L2_CID_VFLIP			0x00980915
#define V4L2_CID_VBLANK			0x009e0901
#define V4L2_CID_HBLANK			0x009e0902
#define V4L2_CID_ANALOGUE_GAIN		0x009e0903
#define V4L2_CID_LINK_FREQ		0x009f0901
#define V4L2_CID_PIXEL_RATE		0x009f0902
#define V4L2_CID_TEST_PATTERN		0x009f0903

#define V4L2_CTRL_TYPE_INTEGER		1
#define V4L2_CTRL_TYPE_MENU		3
#define V4L2_CTRL_TYPE_INTEGER64	5
#define V4L2_CTRL_TYPE_INTEGER_MENU	9

#define V4L2_CTRL_FLAG_GRABBED		0x0002
#define V4L2_CTRL_FLAG_READ_ONLY	0x0004
#define V4L2_CTRL_FLAG_INACTIVE		0x0010
#define V4L2_CTRL_FLAG_VOLATILE		0x0080
#define V4L2_CTRL_FLAG_EXECUTE_ON_WRITE	0x0200
#define V4L2_CTRL_FLAG_MODIFY_LAYOUT	0x0400

#define V4L2_EVENT_CTRL			3
#define V4L2_EVENT_CTRL_CH_VALUE	(1 << 0)
#define V4L2_EVENT_CTRL_CH_FLAGS	(1 << 1)
#define V4L2_EVENT_CTRL_CH_RANGE	(1 << 2)
#define V4L2_EVENT_SUB_FL_SEND_INITIAL	(1 << 0)

struct v4l2_ctrl;

struct v4l2_ctrl_ops {
	int (*g_volatile_ctrl)(struct v4l2_ctrl *ctrl);
	int (*try_ctrl)(struct v4l2_ctrl *ctrl);
	int (*s_ctrl)(struct v4l2_ctrl *ctrl);
};

struct v4l2_ctrl {
	const struct v4l2_ctrl_ops *ops;
	u32 id;
	const char *name;
	int type;
	u32 flags;
	s64 minimum;
	s64 maximum;
	u64 step;
	s64 default_value;
	u64 menu_skip_mask;
	const char *const *qmenu;
	const s64 *qmenu_int;
	s32 val;
	s64 val64;
	union {
		s64 *p_s64;
	} p_new;
	/* The value in effect: the test copies val into it after a set, as the framework does. */
	struct {
		s32 val;
	} cur;
	/* The cluster the control is in, its own alone by default; the framework calls the first. */
	struct v4l2_ctrl **cluster;
	struct v4l2_ctrl *self;
	u32 ncontrols;
	bool has_changed;
	/* A client subscribed to the control's V4L2_EVENT_CTRL. */
	bool subscribed;
	struct v4l2_ctrl_handler *handler;
	void *priv;
	struct v4l2_ctrl *next;
};

/* One V4L2_EVENT_CTRL as a subscriber receives it: the value and the range in effect. */
struct kstub_event {
	u32 id;
	u32 changes;
	s32 value;
	s64 minimum;
	s64 maximum;
	s64 default_value;
};

#define KSTUB_EVENTS_MAX	64

/* The events subscribers received since kstub_events_clear(), the oldest first. */
extern struct kstub_event kstub_events[KSTUB_EVENTS_MAX];
extern unsigned int kstub_event_count;
void kstub_events_clear(void);
/*
 * As the kernel's send_initial_event(), under the handler's lock: a
 * subscription with V4L2_EVENT_SUB_FL_SEND_INITIAL hears CH_FLAGS and
 * CH_VALUE at once, the value in effect with no read of a volatile control.
 */
void kstub_send_initial_event(struct v4l2_ctrl *ctrl);

struct v4l2_ctrl_config {
	const struct v4l2_ctrl_ops *ops;
	u32 id;
	const char *name;
	int type;
	s64 min;
	s64 max;
	u64 step;
	s64 def;
	u32 flags;
	u64 menu_skip_mask;
	const char *const *qmenu;
	const s64 *qmenu_int;
};

/*
 * The handler's lock is its own _lock until the driver points it elsewhere.
 * A handler the test zeroes takes its own at its first control.
 */
struct v4l2_ctrl_handler {
	int error;
	struct v4l2_ctrl *ctrls;
	struct mutex _lock;
	struct mutex *lock;
};

#define v4l2_ctrl_handler_init(hdl, nr)	kstub_ctrl_handler_init(hdl)
#define v4l2_ctrl_handler_free(hdl)	kstub_ctrl_handler_free(hdl)

struct v4l2_ctrl *v4l2_ctrl_new_custom(struct v4l2_ctrl_handler *hdl,
				       const struct v4l2_ctrl_config *cfg, void *priv);
struct v4l2_ctrl *v4l2_ctrl_new_std_menu_items(struct v4l2_ctrl_handler *hdl,
					       const struct v4l2_ctrl_ops *ops, u32 id, u8 max,
					       u64 mask, u8 def, const char *const *qmenu);
struct v4l2_ctrl *v4l2_ctrl_new_std(struct v4l2_ctrl_handler *hdl, const struct v4l2_ctrl_ops *ops,
				    u32 id, s64 min, s64 max, u64 step, s64 def);
struct v4l2_ctrl *v4l2_ctrl_new_int_menu(struct v4l2_ctrl_handler *hdl,
					 const struct v4l2_ctrl_ops *ops, u32 id, u8 max, u8 def,
					 const s64 *qmenu_int);
void v4l2_ctrl_cluster(unsigned int ncontrols, struct v4l2_ctrl **controls);
void __v4l2_ctrl_grab(struct v4l2_ctrl *ctrl, bool grabbed);
int __v4l2_ctrl_modify_range(struct v4l2_ctrl *ctrl, s64 min, s64 max, u64 step, s64 def);
int __v4l2_ctrl_s_ctrl(struct v4l2_ctrl *ctrl, s32 val);
int __v4l2_ctrl_s_ctrl_int64(struct v4l2_ctrl *ctrl, s64 val);
void v4l2_ctrl_activate(struct v4l2_ctrl *ctrl, bool active);
/* The handler's lock, as the kernel's inline forms take it: the abort line names the caller. */
#define v4l2_ctrl_lock(ctrl)	kstub_mutex_lock((ctrl)->handler->lock, __func__)
#define v4l2_ctrl_unlock(ctrl)	kstub_mutex_unlock((ctrl)->handler->lock, __func__)
/* The control of @id on @hdl, NULL for none. */
struct v4l2_ctrl *kstub_ctrl_find(const struct v4l2_ctrl_handler *hdl, u32 id);
/* The controls on @hdl. */
unsigned int kstub_ctrl_count(const struct v4l2_ctrl_handler *hdl);
/*
 * A set of an INTEGER control from user space, as the framework runs it:
 * refused on a read-only or a grabbed control, the value clamped into the
 * range on its step, then under the handler's lock the cluster's master's
 * s_ctrl only when a value changes. A set that succeeds makes the values
 * current and raises one V4L2_EVENT_CTRL per changed control a client
 * subscribed to.
 */
int kstub_ctrl_set(struct v4l2_ctrl *ctrl, s32 val);
/*
 * A read of an INTEGER control from user space, as the framework runs it:
 * under the handler's lock, a volatile control's g_volatile_ctrl on its
 * current value, then the value it leaves.
 */
int kstub_ctrl_get(struct v4l2_ctrl *ctrl, s32 *val);
/* The controls a cluster may hold in one set. */
#define KSTUB_CLUSTER_MAX	8
/*
 * One set of @n controls of one cluster from user space, as one
 * VIDIOC_S_EXT_CTRLS: each refused or clamped as kstub_ctrl_set() does, then
 * one s_ctrl of the master for them all.
 */
int kstub_ctrls_set(struct v4l2_ctrl *const *ctrls, const s32 *vals, unsigned int n);
/*
 * One try of @n controls of one cluster from user space, as one
 * VIDIOC_TRY_EXT_CTRLS: each refused or clamped as a set is, a grabbed
 * control taken too, then the master's try_ctrl alone. No value changes.
 */
int kstub_ctrls_try(struct v4l2_ctrl *const *ctrls, const s32 *vals, unsigned int n);
int kstub_ctrl_handler_init(struct v4l2_ctrl_handler *hdl);
void kstub_ctrl_handler_free(struct v4l2_ctrl_handler *hdl);

#endif /* NXS_KSTUB_H */
