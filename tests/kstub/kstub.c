// SPDX-License-Identifier: GPL-2.0-only
/*
 * kstub.c - the kernel interfaces of kstub.h, for a user-space test
 *
 * Copyright (C) 2026 Aliensense
 */

#include "kstub.h"

/* Set by the test: log every line the core prints. */
bool kstub_verbose;
unsigned int kstub_kvcalloc_failures;

/* The lines the core printed since the last kstub_log_clear(), the oldest first. */
static char kstub_lines[8192];
static size_t kstub_lines_len;

void kstub_log(const struct device *dev, const char *level, const char *fmt, ...)
{
	va_list ap;
	int n;

	/* Each line opens with its level, "[warn] " for one, so a test tells a warning from a note. */
	n = snprintf(kstub_lines + kstub_lines_len, sizeof(kstub_lines) - kstub_lines_len, "[%s] ", level);
	if (n > 0)
		kstub_lines_len += min((size_t)n, sizeof(kstub_lines) - 1 - kstub_lines_len);
	va_start(ap, fmt);
	n = vsnprintf(kstub_lines + kstub_lines_len, sizeof(kstub_lines) - kstub_lines_len, fmt, ap);
	va_end(ap);
	if (n > 0)
		kstub_lines_len += min((size_t)n, sizeof(kstub_lines) - 1 - kstub_lines_len);
	if (!kstub_verbose)
		return;
	fprintf(stderr, "  [%s] %s: ", level, dev && dev->name ? dev->name : "dev");
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
}

void kstub_log_clear(void)
{
	kstub_lines_len = 0;
	kstub_lines[0] = 0;
}

bool kstub_log_holds(const char *text)
{
	return strstr(kstub_lines, text) != NULL;
}

/* ---- mutexes ---------------------------------------------------------- */

void kstub_mutex_init(struct mutex *lock, const char *name)
{
	lock->name = name;
	lock->holder = NULL;
	lock->takes = 0;
}

void kstub_mutex_lock(struct mutex *lock, const char *func)
{
	if (lock->holder) {
		fprintf(stderr, "kstub: %s takes mutex %s, which %s holds: a kernel mutex is not recursive\n",
			func, lock->name, lock->holder);
		abort();
	}
	lock->holder = func;
	lock->takes++;
}

void kstub_mutex_unlock(struct mutex *lock, const char *func)
{
	if (!lock->holder) {
		fprintf(stderr, "kstub: %s releases mutex %s, which no one holds\n", func,
			lock->name);
		abort();
	}
	lock->holder = NULL;
}

void kstub_lockdep_assert_held(const struct mutex *lock, const char *func)
{
	if (lock->holder)
		return;
	fprintf(stderr, "kstub: %s asserts mutex %s held, and no one holds it\n", func, lock->name);
	abort();
}

/* ---- device-managed resources ----------------------------------------- */

struct kstub_res {
	struct device *dev;
	void *mem;
	void (*action)(void *);
	void *data;
	struct kstub_res *next;
};

static struct kstub_res *kstub_res;

static void *kstub_res_add(struct device *dev, void *mem, void (*action)(void *), void *data)
{
	struct kstub_res *r = calloc(1, sizeof(*r));

	if (!r)
		return NULL;
	r->dev = dev;
	r->mem = mem;
	r->action = action;
	r->data = data;
	r->next = kstub_res;
	kstub_res = r;

	return r;
}

void *devm_kcalloc(struct device *dev, size_t n, size_t size, int gfp)
{
	void *mem = calloc(n ? n : 1, size ? size : 1);

	(void)gfp;
	if (mem && !kstub_res_add(dev, mem, NULL, NULL)) {
		free(mem);
		return NULL;
	}

	return mem;
}

void *devm_kzalloc(struct device *dev, size_t size, int gfp)
{
	return devm_kcalloc(dev, 1, size, gfp);
}

void *devm_kmemdup(struct device *dev, const void *src, size_t len, int gfp)
{
	void *mem = devm_kcalloc(dev, 1, len, gfp);

	if (mem)
		memcpy(mem, src, len);

	return mem;
}

char *devm_kasprintf(struct device *dev, int gfp, const char *fmt, ...)
{
	char buf[128];
	va_list ap;
	char *s;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
	s = devm_kcalloc(dev, strlen(buf) + 1, 1, gfp);
	if (s)
		strcpy(s, buf);

	return s;
}

int devm_add_action_or_reset(struct device *dev, void (*action)(void *), void *data)
{
	if (!kstub_res_add(dev, NULL, action, data)) {
		action(data);
		return -ENOMEM;
	}

	return 0;
}

void kstub_devm_release(struct device *dev)
{
	struct kstub_res **p = &kstub_res;

	while (*p) {
		struct kstub_res *r = *p;

		if (r->dev != dev) {
			p = &r->next;
			continue;
		}
		*p = r->next;
		if (r->action)
			r->action(r->data);
		free(r->mem);
		free(r);
	}
}

/* ---- I2C -------------------------------------------------------------- */

/* A read past the adapter's max_read_len, which the I2C core refuses before the wire. */
static bool kstub_quirk(struct i2c_adapter *adap, const struct i2c_msg *msgs, int num)
{
	const struct i2c_adapter_quirks *q = adap->quirks;
	int i;

	for (i = 0; q && q->max_read_len && i < num; i++) {
		if ((msgs[i].flags & I2C_M_RD) && msgs[i].len > q->max_read_len) {
			adap->quirk_errors++;
			return true;
		}
	}

	return false;
}

int i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	if (adap->locked)
		adap->lock_errors++;
	if (kstub_quirk(adap, msgs, num))
		return -EOPNOTSUPP;

	return adap->xfer(adap->fake, msgs, num, false);
}

int __i2c_transfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	if (!adap->locked)
		adap->lock_errors++;
	if (kstub_quirk(adap, msgs, num))
		return -EOPNOTSUPP;

	return adap->xfer(adap->fake, msgs, num, true);
}

void i2c_lock_bus(struct i2c_adapter *adap, unsigned int flags)
{
	(void)flags;
	if (adap->locked)
		adap->lock_errors++;
	adap->locked++;
}

void i2c_unlock_bus(struct i2c_adapter *adap, unsigned int flags)
{
	(void)flags;
	if (!adap->locked)
		adap->lock_errors++;
	else
		adap->locked--;
}

/* ---- V4L2 controls ---------------------------------------------------- */

static struct v4l2_ctrl *kstub_ctrl_add(struct v4l2_ctrl_handler *hdl,
					const struct v4l2_ctrl_ops *ops, u32 id, const char *name,
					int type, s64 min, s64 max, u64 step, s64 def)
{
	struct v4l2_ctrl *ctrl = calloc(1, sizeof(*ctrl));

	if (!hdl->lock)
		kstub_ctrl_handler_init(hdl);
	if (!ctrl) {
		hdl->error = -ENOMEM;
		return NULL;
	}
	ctrl->ops = ops;
	ctrl->id = id;
	ctrl->name = name;
	ctrl->type = type;
	ctrl->minimum = min;
	ctrl->maximum = max;
	ctrl->step = step;
	ctrl->default_value = def;
	ctrl->val = (s32)def;
	ctrl->val64 = def;
	ctrl->p_new.p_s64 = &ctrl->val64;
	ctrl->cur.val = (s32)def;
	ctrl->self = ctrl;
	ctrl->cluster = &ctrl->self;
	ctrl->ncontrols = 1;
	ctrl->handler = hdl;
	ctrl->next = hdl->ctrls;
	hdl->ctrls = ctrl;

	return ctrl;
}

struct v4l2_ctrl *v4l2_ctrl_new_custom(struct v4l2_ctrl_handler *hdl,
				       const struct v4l2_ctrl_config *cfg, void *priv)
{
	struct v4l2_ctrl *ctrl = kstub_ctrl_add(hdl, cfg->ops, cfg->id, cfg->name, cfg->type,
						cfg->min, cfg->max, cfg->step, cfg->def);

	if (!ctrl)
		return NULL;
	ctrl->flags = cfg->flags;
	ctrl->menu_skip_mask = cfg->menu_skip_mask;
	ctrl->qmenu = cfg->qmenu;
	ctrl->qmenu_int = cfg->qmenu_int;
	ctrl->priv = priv;

	return ctrl;
}

struct v4l2_ctrl *v4l2_ctrl_new_std_menu_items(struct v4l2_ctrl_handler *hdl,
					       const struct v4l2_ctrl_ops *ops, u32 id, u8 max,
					       u64 mask, u8 def, const char *const *qmenu)
{
	struct v4l2_ctrl *ctrl = kstub_ctrl_add(hdl, ops, id, "Test Pattern", V4L2_CTRL_TYPE_MENU,
						0, max, 1, def);

	if (!ctrl)
		return NULL;
	ctrl->menu_skip_mask = mask;
	ctrl->qmenu = qmenu;

	return ctrl;
}

struct v4l2_ctrl *v4l2_ctrl_new_std(struct v4l2_ctrl_handler *hdl, const struct v4l2_ctrl_ops *ops,
				    u32 id, s64 min, s64 max, u64 step, s64 def)
{
	return kstub_ctrl_add(hdl, ops, id, "Standard", V4L2_CTRL_TYPE_INTEGER, min, max, step, def);
}

struct v4l2_ctrl *v4l2_ctrl_new_int_menu(struct v4l2_ctrl_handler *hdl,
					 const struct v4l2_ctrl_ops *ops, u32 id, u8 max, u8 def,
					 const s64 *qmenu_int)
{
	struct v4l2_ctrl *ctrl = kstub_ctrl_add(hdl, ops, id, "Integer Menu",
						V4L2_CTRL_TYPE_INTEGER_MENU, 0, max, 1, def);

	if (ctrl)
		ctrl->qmenu_int = qmenu_int;

	return ctrl;
}

void v4l2_ctrl_cluster(unsigned int ncontrols, struct v4l2_ctrl **controls)
{
	unsigned int i;

	for (i = 0; i < ncontrols; i++) {
		controls[i]->cluster = controls;
		controls[i]->ncontrols = ncontrols;
	}
}

void __v4l2_ctrl_grab(struct v4l2_ctrl *ctrl, bool grabbed)
{
	if (grabbed)
		ctrl->flags |= V4L2_CTRL_FLAG_GRABBED;
	else
		ctrl->flags &= ~V4L2_CTRL_FLAG_GRABBED;
}


int __v4l2_ctrl_s_ctrl_int64(struct v4l2_ctrl *ctrl, s64 val)
{
	ctrl->val64 = val;

	return ctrl->ops && ctrl->ops->s_ctrl ? ctrl->ops->s_ctrl(ctrl) : 0;
}

struct kstub_event kstub_events[KSTUB_EVENTS_MAX];
unsigned int kstub_event_count;

void kstub_events_clear(void)
{
	kstub_event_count = 0;
}

static void kstub_send_event(const struct v4l2_ctrl *ctrl, u32 changes)
{
	if (!ctrl->subscribed || kstub_event_count >= KSTUB_EVENTS_MAX)
		return;
	kstub_events[kstub_event_count].id = ctrl->id;
	kstub_events[kstub_event_count].changes = changes;
	kstub_events[kstub_event_count].value = ctrl->cur.val;
	kstub_events[kstub_event_count].minimum = ctrl->minimum;
	kstub_events[kstub_event_count].maximum = ctrl->maximum;
	kstub_events[kstub_event_count].default_value = ctrl->default_value;
	kstub_event_count++;
}

void kstub_send_initial_event(struct v4l2_ctrl *ctrl)
{
	mutex_lock(ctrl->handler->lock);
	kstub_send_event(ctrl, V4L2_EVENT_CTRL_CH_FLAGS | V4L2_EVENT_CTRL_CH_VALUE);
	mutex_unlock(ctrl->handler->lock);
}

/* As the kernel's: a change of the inactive flag raises CH_FLAGS. */
void v4l2_ctrl_activate(struct v4l2_ctrl *ctrl, bool active)
{
	bool was = ctrl->flags & V4L2_CTRL_FLAG_INACTIVE;

	if (active)
		ctrl->flags &= ~V4L2_CTRL_FLAG_INACTIVE;
	else
		ctrl->flags |= V4L2_CTRL_FLAG_INACTIVE;
	if (was == active)
		kstub_send_event(ctrl, V4L2_EVENT_CTRL_CH_FLAGS);
}

/* The kernel's ROUND_TO_RANGE: to the nearest step from the minimum, inside the range. */
static s32 kstub_round_to_range(const struct v4l2_ctrl *ctrl, s64 val)
{
	u64 step = ctrl->step ? ctrl->step : 1;

	if (ctrl->maximum >= 0 && val >= ctrl->maximum - (s64)(step / 2))
		val = ctrl->maximum;
	else
		val += step / 2;
	val = clamp_t(s64, val, ctrl->minimum, ctrl->maximum);

	return ctrl->minimum + step * ((u64)(val - ctrl->minimum) / step);
}

/*
 * The framework's try or set of one cluster under its handler's lock: the
 * controls of @ctrls take @vals and the others keep their values. The
 * master's try_ctrl runs first, and a try ends there. A volatile control
 * never counts as changed. The master's s_ctrl runs when a value changes,
 * and a set that succeeds makes the new values current and raises
 * V4L2_EVENT_CTRL: CH_VALUE on each changed control and @ch_flags on every
 * control of the cluster.
 */
static int kstub_try_or_set_cluster(struct v4l2_ctrl *const *ctrls, const s32 *vals,
				    unsigned int n, bool set, u32 ch_flags)
{
	struct v4l2_ctrl *master = ctrls[0]->cluster[0];
	bool changed = false;
	unsigned int i, j;
	int err;

	for (i = 0; i < master->ncontrols; i++) {
		struct v4l2_ctrl *member = master->cluster[i];

		member->val = member->cur.val;
		for (j = 0; j < n; j++)
			if (ctrls[j] == member)
				member->val = vals[j];
		member->has_changed = !(member->flags & V4L2_CTRL_FLAG_VOLATILE) &&
				      member->val != member->cur.val;
		changed |= member->has_changed;
	}
	err = master->ops && master->ops->try_ctrl ? master->ops->try_ctrl(master) : 0;
	if (err || !set || !changed)
		return err;
	err = master->ops && master->ops->s_ctrl ? master->ops->s_ctrl(master) : 0;
	if (err)
		return err;
	for (i = 0; i < master->ncontrols; i++) {
		struct v4l2_ctrl *member = master->cluster[i];
		u32 changes = ch_flags;

		if (member->has_changed) {
			member->cur.val = member->val;
			changes |= V4L2_EVENT_CTRL_CH_VALUE;
		}
		if (changes)
			kstub_send_event(member, changes);
	}

	return 0;
}

static int kstub_set_cluster(struct v4l2_ctrl *const *ctrls, const s32 *vals, unsigned int n,
			     u32 ch_flags)
{
	return kstub_try_or_set_cluster(ctrls, vals, n, true, ch_flags);
}

/* The value a set brings an INTEGER control, on its step inside its range; another type's as it is. */
static s32 kstub_validate(const struct v4l2_ctrl *ctrl, s64 val)
{
	return ctrl->type == V4L2_CTRL_TYPE_INTEGER ? kstub_round_to_range(ctrl, val) : (s32)val;
}

static int kstub_ctrls_try_or_set(struct v4l2_ctrl *const *ctrls, const s32 *vals, unsigned int n,
				  bool set)
{
	s32 valid[KSTUB_CLUSTER_MAX];
	unsigned int i;
	int err;

	if (!n || n > ARRAY_SIZE(valid))
		return -EINVAL;
	for (i = 0; i < n; i++) {
		if (ctrls[i]->cluster[0] != ctrls[0]->cluster[0])
			return -EINVAL;
		if (ctrls[i]->flags & V4L2_CTRL_FLAG_READ_ONLY)
			return -EACCES;
		if (set && (ctrls[i]->flags & V4L2_CTRL_FLAG_GRABBED))
			return -EBUSY;
		valid[i] = kstub_round_to_range(ctrls[i], vals[i]);
	}
	mutex_lock(ctrls[0]->handler->lock);
	err = kstub_try_or_set_cluster(ctrls, valid, n, set, 0);
	mutex_unlock(ctrls[0]->handler->lock);

	return err;
}

int kstub_ctrls_set(struct v4l2_ctrl *const *ctrls, const s32 *vals, unsigned int n)
{
	return kstub_ctrls_try_or_set(ctrls, vals, n, true);
}

int kstub_ctrls_try(struct v4l2_ctrl *const *ctrls, const s32 *vals, unsigned int n)
{
	return kstub_ctrls_try_or_set(ctrls, vals, n, false);
}

int kstub_ctrl_set(struct v4l2_ctrl *ctrl, s32 val)
{
	return kstub_ctrls_set(&ctrl, &val, 1);
}

/*
 * As the kernel's: a range that leaves an INTEGER control's value outside
 * is a set of the value on its step inside the new range, raising CH_VALUE
 * and CH_RANGE through its cluster's set; a range that moved alone raises
 * CH_RANGE on the control.
 */
int __v4l2_ctrl_modify_range(struct v4l2_ctrl *ctrl, s64 min, s64 max, u64 step, s64 def)
{
	bool moved = ctrl->minimum != min || ctrl->maximum != max || ctrl->step != step ||
		     ctrl->default_value != def;
	s32 val;

	ctrl->minimum = min;
	ctrl->maximum = max;
	ctrl->step = step;
	ctrl->default_value = def;
	val = kstub_validate(ctrl, ctrl->cur.val);
	if (val != ctrl->cur.val)
		return kstub_set_cluster(&ctrl, &val, 1, V4L2_EVENT_CTRL_CH_RANGE);
	if (moved)
		kstub_send_event(ctrl, V4L2_EVENT_CTRL_CH_RANGE);

	return 0;
}

/* As the kernel's: the value on its step inside the range, set through the control's cluster. */
int __v4l2_ctrl_s_ctrl(struct v4l2_ctrl *ctrl, s32 val)
{
	s32 valid = kstub_validate(ctrl, val);

	return kstub_set_cluster(&ctrl, &valid, 1, 0);
}

int kstub_ctrl_get(struct v4l2_ctrl *ctrl, s32 *val)
{
	int err = 0;

	mutex_lock(ctrl->handler->lock);
	ctrl->val = ctrl->cur.val;
	if ((ctrl->flags & V4L2_CTRL_FLAG_VOLATILE) && ctrl->ops && ctrl->ops->g_volatile_ctrl)
		err = ctrl->ops->g_volatile_ctrl(ctrl);
	*val = ctrl->val;
	mutex_unlock(ctrl->handler->lock);

	return err;
}

unsigned int kstub_ctrl_count(const struct v4l2_ctrl_handler *hdl)
{
	const struct v4l2_ctrl *ctrl;
	unsigned int n = 0;

	for (ctrl = hdl->ctrls; ctrl; ctrl = ctrl->next)
		n++;

	return n;
}

struct v4l2_ctrl *kstub_ctrl_find(const struct v4l2_ctrl_handler *hdl, u32 id)
{
	struct v4l2_ctrl *ctrl;

	for (ctrl = hdl->ctrls; ctrl; ctrl = ctrl->next)
		if (ctrl->id == id)
			return ctrl;

	return NULL;
}

/* As the kernel's: the handler's lock is its own until the driver points it at another. */
int kstub_ctrl_handler_init(struct v4l2_ctrl_handler *hdl)
{
	hdl->error = 0;
	hdl->ctrls = NULL;
	hdl->lock = &hdl->_lock;
	mutex_init(hdl->lock);

	return 0;
}

void kstub_ctrl_handler_free(struct v4l2_ctrl_handler *hdl)
{
	while (hdl->ctrls) {
		struct v4l2_ctrl *next = hdl->ctrls->next;

		free(hdl->ctrls);
		hdl->ctrls = next;
	}
}
