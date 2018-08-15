#include <psp2kern/kernel/modulemgr.h>
#include <psp2kern/kernel/threadmgr.h>
#include <psp2kern/kernel/sysmem.h>
#include <psp2kern/kernel/suspend.h>
#include <psp2kern/bt.h>
#include <psp2kern/ctrl.h>
#include <psp2/touch.h>
#include <taihen.h>
#include <math.h>
#include "log.h"

/*
 * Needed by newlib's libm.
 */
int __errno;

#define DS3_VID 0x054C
#define DS3_PID 0x0268

#define DS3_JOYSTICK_THRESHOLD 5
#define DS3_TRIGGER_THRESHOLD 1

#define EVF_EXIT	(1 << 0)

#define abs(x) (((x) < 0) ? -(x) : (x))

#define DECL_FUNC_HOOK(name, ...) \
	static tai_hook_ref_t name##_ref; \
	static SceUID name##_hook_uid = -1; \
	static int name##_hook_func(__VA_ARGS__)
 #define BIND_FUNC_OFFSET_HOOK(name, pid, modid, segidx, offset, thumb) \
	name##_hook_uid = taiHookFunctionOffsetForKernel((pid), \
		&name##_ref, (modid), (segidx), (offset), thumb, name##_hook_func)
 #define BIND_FUNC_EXPORT_HOOK(name, pid, module, lib_nid, func_nid) \
	name##_hook_uid = taiHookFunctionExportForKernel((pid), \
		&name##_ref, (module), (lib_nid), (func_nid), name##_hook_func)
 #define UNBIND_FUNC_HOOK(name) \
	do { \
		if (name##_hook_uid > 0) \
			taiHookReleaseForKernel(name##_hook_uid, name##_ref); \
	} while(0)

struct ds3_input_report {
	unsigned char report_id;
	unsigned char unk0;

	unsigned char select : 1;
	unsigned char l3     : 1;
	unsigned char r3     : 1;
	unsigned char start  : 1;
	unsigned char up     : 1;
	unsigned char right  : 1;
	unsigned char down   : 1;
	unsigned char left   : 1;

	unsigned char l2       : 1;
	unsigned char r2       : 1;
	unsigned char l1       : 1;
	unsigned char r1       : 1;
	unsigned char triangle : 1;
	unsigned char circle   : 1;
	unsigned char cross    : 1;
	unsigned char square   : 1;

	unsigned char ps       : 1;
	unsigned char not_used : 7;

	unsigned char unk1;

	unsigned char left_x;
	unsigned char left_y;
	unsigned char right_x;
	unsigned char right_y;

	unsigned int unk2;

	unsigned char up_sens;
	unsigned char right_sens;
	unsigned char down_sens;
	unsigned char left_sens;

	unsigned char L2_sens;
	unsigned char R2_sens;
	unsigned char L1_sens;
	unsigned char R1_sens;

	unsigned char triangle_sens;
	unsigned char circle_sens;
	unsigned char cross_sens;
	unsigned char square_sens;

	unsigned short unk3;
	unsigned char unk4;

	unsigned char status;
	unsigned char power_rating;
	unsigned char comm_status;
	unsigned int unk5;
	unsigned int unk6;
	unsigned char unk7;

	unsigned short accel_x;
	unsigned short accel_y;
	unsigned short accel_z;

	union {
		unsigned short gyro_z;
		unsigned short roll;
	};
} __attribute__((packed, aligned(32)));

static SceUID bt_mempool_uid = -1;
static SceUID bt_thread_evflag_uid = -1;
static SceUID bt_thread_uid = -1;
static SceUID bt_cb_uid = -1;
static int bt_thread_run = 1;

static int ds3_connected = 0;
static unsigned int ds3_mac0 = 0;
static unsigned int ds3_mac1 = 0;

static struct ds3_input_report ds3_input;

/*static SceUID SceBt_sub_2292CE4_0x2292D18_patch_uid = -1;
static tai_hook_ref_t SceBt_sub_22999C8_ref;*/
static SceUID SceBt_sub_22999C8_hook_uid = -1;
static tai_hook_ref_t SceBt_sub_22999C8_ref;
static SceUID SceBt_sub_22947E4_hook_uid = -1;
static tai_hook_ref_t SceBt_sub_22947E4_ref;

static inline void ds3_input_reset(void)
{
	memset(&ds3_input, 0, sizeof(ds3_input));
}

static int is_ds3(const unsigned short vid_pid[2])
{
	return vid_pid[0] == DS3_VID && vid_pid[1] == DS3_PID;
}

static inline void *mempool_alloc(unsigned int size)
{
	return ksceKernelAllocHeapMemory(bt_mempool_uid, size);
}

static inline void mempool_free(void *ptr)
{
	ksceKernelFreeHeapMemory(bt_mempool_uid, ptr);
}

static int ds3_send_report(unsigned int mac0, unsigned int mac1, uint8_t flags, uint8_t report,
			    size_t len, const void *data)
{
	SceBtHidRequest *req;
	unsigned char *buf;

	req = mempool_alloc(sizeof(*req));
	if (!req) {
		LOG("Error allocatin BT HID Request\n");
		return -1;
	}

	if ((buf = mempool_alloc((len + 1) * sizeof(*buf))) == NULL) {
		LOG("Memory allocation error (mesg array)\n");
		return -1;
	}

	buf[0] = report;
	memcpy(buf + 1, data, len);

	memset(req, 0, sizeof(*req));
	req->type = 1; // 0xA2 -> type = 1
	req->buffer = buf;
	req->length = len + 1;
	req->next = req;

	TEST_CALL(ksceBtHidTransfer, mac0, mac1, req);

	mempool_free(buf);
	mempool_free(req);

	return 0;
}

static int ds3_send_feature_report(unsigned int mac0, unsigned int mac1, uint8_t flags, uint8_t report,
			    size_t len, const void *data)
{
	SceBtHidRequest *req;
	unsigned char *buf;

	req = mempool_alloc(sizeof(*req));
	if (!req) {
		LOG("Error allocatin BT HID Request\n");
		return -1;
	}

	if ((buf = mempool_alloc((len + 1) * sizeof(*buf))) == NULL) {
		LOG("Memory allocation error (mesg array)\n");
		return -1;
	}

	buf[0] = report;
	memcpy(buf + 1, data, len);

	memset(req, 0, sizeof(*req));
	req->type = 3; // 0x53 -> type = 3
	req->buffer = buf;
	req->length = len + 1;
	req->next = req;

	TEST_CALL(ksceBtHidTransfer, mac0, mac1, req);

	mempool_free(buf);
	mempool_free(req);

	return 0;
}


static int ds3_send_leds_rumble(unsigned int mac0, unsigned int mac1)
{
	static const unsigned char led_pattern[] = {
		0x0, 0x02, 0x04, 0x08, 0x10, 0x12, 0x14, 0x18, 0x1A, 0x1C, 0x1E
	};

	unsigned char buf[] = {
		0x01,
		0x00, //Padding
		0x00, 0x00, 0x00, 0x00, //Rumble (r, r, l, l)
		0x00, 0x00, 0x00, 0x00, //Padding
		0x00, /* LED_1 = 0x02, LED_2 = 0x04, ... */
		0xff, 0x27, 0x10, 0x00, 0x32, /* LED_4 */
		0xff, 0x27, 0x10, 0x00, 0x32, /* LED_3 */
		0xff, 0x27, 0x10, 0x00, 0x32, /* LED_2 */
		0xff, 0x27, 0x10, 0x00, 0x32, /* LED_1 */
		0x00, 0x00, 0x00, 0x00, 0x00  /* LED_5 (not soldered) */
	};

	buf[8] = led_pattern[1]; /* Turn on LED 1 */

	if (ds3_send_report(mac0, mac1, 0, 0x01, sizeof(buf), buf)) {
		LOG("Send report error\n");
		return -1;
	}

	return 0;
}

static int ds3_set_operational(unsigned int mac0, unsigned int mac1)
{
	unsigned char data[] = {
		0x42, 0x03, 0x00, 0x00
	};

	if (ds3_send_feature_report(mac0, mac1, 0, 0xF4, sizeof(data), data)) {
		LOG("Set operational error\n");
		return -1;
	}

	return 0;
}


DECL_FUNC_HOOK(SceCtrl_ksceCtrlGetControllerPortInfo, SceCtrlPortInfo *info)
{
	int ret = TAI_CONTINUE(int, SceCtrl_ksceCtrlGetControllerPortInfo_ref, info);

	if (ret >= 0 && ds3_connected) {
		// info->port[0] |= SCE_CTRL_TYPE_VIRT;
		info->port[1] = SCE_CTRL_TYPE_DS3;
	}

	return ret;
}

static void patch_ctrl_data(const struct ds3_input_report *ds3, SceCtrlData *pad_data)
{
	signed char ldx, ldy, rdx, rdy;
	unsigned int buttons = 0;
	int left_js_moved = 0;
	int right_js_moved = 0;

	if (ds3->cross)
		buttons |= SCE_CTRL_CROSS;
	if (ds3->circle)
		buttons |= SCE_CTRL_CIRCLE;
	if (ds3->triangle)
		buttons |= SCE_CTRL_TRIANGLE;
	if (ds3->square)
		buttons |= SCE_CTRL_SQUARE;

	if (ds3->up)
		buttons |= SCE_CTRL_UP;
	if (ds3->right)
		buttons |= SCE_CTRL_RIGHT;
	if (ds3->down)
		buttons |= SCE_CTRL_DOWN;
	if (ds3->left)
		buttons |= SCE_CTRL_LEFT;

	if (ds3->l1)
		buttons |= SCE_CTRL_L1;
	if (ds3->r1)
		buttons |= SCE_CTRL_R1;

	if (ds3->l2)
		buttons |= SCE_CTRL_LTRIGGER;
	if (ds3->r2)
		buttons |= SCE_CTRL_RTRIGGER;

	if (ds3->l3)
		buttons |= SCE_CTRL_L3;
	if (ds3->r3)
		buttons |= SCE_CTRL_R3;

	if (ds3->select)
		buttons |= SCE_CTRL_SELECT;
	if (ds3->start)
		buttons |= SCE_CTRL_START;
	if (ds3->ps)
		buttons |= SCE_CTRL_INTERCEPTED;

	ldx = ds3->left_x - 128;
	ldy = ds3->left_y - 128;
	rdx = ds3->right_x - 128;
	rdy = ds3->right_y - 128;

	if (sqrtf(ldx * ldx + ldy * ldy) > DS3_JOYSTICK_THRESHOLD)
		left_js_moved = 1;

 	if (sqrtf(rdx * rdx + rdy * rdy) > DS3_JOYSTICK_THRESHOLD)
		right_js_moved = 1;

	if (left_js_moved) {
		pad_data->lx = ds3->left_x;
		pad_data->ly = ds3->left_y;
	}

	if (right_js_moved) {
		pad_data->rx = ds3->right_x;
		pad_data->ry = ds3->right_y;
	}

	if (ds3->L2_sens > DS3_TRIGGER_THRESHOLD)
		pad_data->lt = ds3->L2_sens;

	if (ds3->R2_sens > DS3_TRIGGER_THRESHOLD)
		pad_data->rt = ds3->R2_sens;

	if (ds3->ps)
		ksceCtrlSetButtonEmulation(0, 0, 0, SCE_CTRL_INTERCEPTED, 16);

	if (buttons != 0 || left_js_moved || right_js_moved ||
	    ds3->L2_sens > DS3_TRIGGER_THRESHOLD ||
	    ds3->R2_sens > DS3_TRIGGER_THRESHOLD)
		ksceKernelPowerTick(0);

	pad_data->buttons |= buttons;
}

static void patch_ctrl_data_all_user(const struct ds3_input_report *ds3,
				     int port, SceCtrlData *pad_data, int count)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		SceCtrlData k_data;

		ksceKernelMemcpyUserToKernel(&k_data, (uintptr_t)pad_data, sizeof(k_data));
		patch_ctrl_data(ds3, &k_data);
		ksceKernelMemcpyKernelToUser((uintptr_t)pad_data, &k_data, sizeof(k_data));

		pad_data++;
	}
}

static void patch_ctrl_data_all_kernel(const struct ds3_input_report *ds3,
				       int port, SceCtrlData *pad_data, int count)
{
	unsigned int i;

	for (i = 0; i < count; i++, pad_data++)
		patch_ctrl_data(ds3, pad_data);
}

#define DECL_FUNC_HOOK_PATCH_CTRL(type, name) \
	DECL_FUNC_HOOK(SceCtrl_##name, int port, SceCtrlData *pad_data, int count) \
	{ \
		int ret = TAI_CONTINUE(int, SceCtrl_ ##name##_ref, port, pad_data, count); \
		if (ret >= 0 && ds3_connected) \
			patch_ctrl_data_all_##type(&ds3_input, port, pad_data, count); \
		return ret; \
	}

DECL_FUNC_HOOK_PATCH_CTRL(kernel, ksceCtrlPeekBufferNegative)
DECL_FUNC_HOOK_PATCH_CTRL(user, sceCtrlPeekBufferNegative2)
DECL_FUNC_HOOK_PATCH_CTRL(kernel, ksceCtrlPeekBufferPositive)
DECL_FUNC_HOOK_PATCH_CTRL(user, sceCtrlPeekBufferPositive2)
DECL_FUNC_HOOK_PATCH_CTRL(user, sceCtrlPeekBufferPositiveExt)
DECL_FUNC_HOOK_PATCH_CTRL(user, sceCtrlPeekBufferPositiveExt2)
DECL_FUNC_HOOK_PATCH_CTRL(kernel, ksceCtrlReadBufferNegative)
DECL_FUNC_HOOK_PATCH_CTRL(user, sceCtrlReadBufferNegative2)
DECL_FUNC_HOOK_PATCH_CTRL(kernel, ksceCtrlReadBufferPositive)
DECL_FUNC_HOOK_PATCH_CTRL(user, sceCtrlReadBufferPositive2)
DECL_FUNC_HOOK_PATCH_CTRL(user, sceCtrlReadBufferPositiveExt)
DECL_FUNC_HOOK_PATCH_CTRL(user, sceCtrlReadBufferPositiveExt2)

static void enqueue_read_request(unsigned int mac0, unsigned int mac1,
				 SceBtHidRequest *request, unsigned char *buffer,
				 unsigned int length)
{
	memset(request, 0, sizeof(*request));
	memset(buffer, 0, length);

	request->type = 0;
	request->buffer = buffer;
	request->length = length;
	request->next = request;

	ksceBtHidTransfer(mac0, mac1, request);
}

static int SceBt_sub_22999C8_hook_func(void *dev_base_ptr, int r1)
{
	unsigned int flags = *(unsigned int *)(r1 + 4);

	if (dev_base_ptr && !(flags & 2)) {
		const void *dev_info = *(const void **)(dev_base_ptr + 0x14A4);
		const unsigned short *vid_pid = (const unsigned short *)(dev_info + 0x28);

		if (is_ds3(vid_pid)) {
			unsigned int *v8_ptr = (unsigned int *)(*(unsigned int *)dev_base_ptr + 8);

			/*
			 * We need to enable the following bits in order to make the Vita
			 * accept the new connection, otherwise it will refuse it.
			 */
			*v8_ptr |= 0x11000;
		}
	}

	return TAI_CONTINUE(int, SceBt_sub_22999C8_ref, dev_base_ptr, r1);
}

static void *SceBt_sub_22947E4_hook_func(unsigned int r0, unsigned int r1, unsigned long long r2)
{
	void *ret = TAI_CONTINUE(void *, SceBt_sub_22947E4_ref, r0, r1, r2);

	if (ret) {
		/*
		 * We have to enable this bit in order to make the Vita
		 * accept the controller.
		 */
		*(unsigned int *)(ret + 0x24) |= 0x1000;
	}

	return ret;
}

static int bt_cb_func(int notifyId, int notifyCount, int notifyArg, void *common)
{
	static SceBtHidRequest hid_request;
	static unsigned char recv_buff[0x100];

	while (1) {
		int ret;
		SceBtEvent hid_event;

		memset(&hid_event, 0, sizeof(hid_event));

		do {
			ret = ksceBtReadEvent(&hid_event, 1);
		} while (ret == SCE_BT_ERROR_CB_OVERFLOW);

		if (ret <= 0) {
			break;
		}

		LOG("->Event:");
		for (int i = 0; i < 0x10; i++)
			LOG(" %02X", hid_event.data[i]);
		LOG("\n");

		/*
		 * If we get an event with a MAC, and the MAC is different
		 * from the connected DS3, skip the event.
		 */
		if (ds3_connected) {
			if (hid_event.mac0 != ds3_mac0 || hid_event.mac1 != ds3_mac1)
				continue;
		}

		switch (hid_event.id) {
		case 0x01: /* Inquiry result event */
			break;

		case 0x02: /* Inquiry stop event */
			break;

		case 0x04: /* Link key request? event */
			break;

		case 0x05: { /* Connection accepted event */
			unsigned short vid_pid[2] = {0, 0};
			char name[0x79];
			unsigned int result1;
			unsigned int result2;

			/*
			 * Getting the VID/PID or device name of the DS3
			 * sometimes? returns an error.
			 */

			result1 = ksceBtGetVidPid(hid_event.mac0, hid_event.mac1, vid_pid);
			result2 = ksceBtGetDeviceName(hid_event.mac0, hid_event.mac1, name);

			if (is_ds3(vid_pid) || (result1 == 0x802F5001 &&
			    result2 == 0x802F0C01)) {
				ds3_input_reset();
				ds3_mac0 = hid_event.mac0;
				ds3_mac1 = hid_event.mac1;
				ds3_connected = 1;
				ds3_set_operational(hid_event.mac0, hid_event.mac1);
				//ds3_send_leds_rumble(hid_event.mac0, hid_event.mac1);
			}
			break;
		}


		case 0x06: /* Device disconnect event*/
			ds3_connected = 0;
			ds3_input_reset();
			break;

		case 0x08: /* Connection requested event */
			/*
			 * Do nothing since we will get a 0x05 event afterwards.
			 */
			break;

		case 0x09: /* Connection request without being paired? event */
			break;

		case 0x0A: /* HID reply to 0-type request */

			LOG("DS3 0x0A event: 0x%02X\n", recv_buff[0]);

			switch (recv_buff[0]) {
			case 0x01: /* Full report */
				/*
				 * Save DS3 state to a global variable.
				 */
				memcpy(&ds3_input, recv_buff, sizeof(ds3_input));

				enqueue_read_request(hid_event.mac0, hid_event.mac1,
					&hid_request, recv_buff, sizeof(recv_buff));
				break;

			default:
				LOG("Unknown DS3 event: 0x%02X\n", recv_buff[0]);
				break;
			}

			break;

		case 0x0B: /* HID reply to 1-type request */

			//LOG("DS3 0x0B event: 0x%02X\n", recv_buff[0]);

			enqueue_read_request(hid_event.mac0, hid_event.mac1,
				&hid_request, recv_buff, sizeof(recv_buff));

			break;

		case 0x0C: /* HID reply to 3-type request? */

			//LOG("DS3 0x0C event: 0x%02X\n", recv_buff[0]);

			enqueue_read_request(hid_event.mac0, hid_event.mac1,
				&hid_request, recv_buff, sizeof(recv_buff));

			break;


		}
	}

	return 0;
}

static int ds3vita_bt_thread(SceSize args, void *argp)
{
	bt_cb_uid = ksceKernelCreateCallback("ds3vita_bt_callback", 0, bt_cb_func, NULL);

	ksceBtRegisterCallback(bt_cb_uid, 0, 0xFFFFFFFF, 0xFFFFFFFF);

/*#ifndef RELEASE
	ksceBtStartInquiry();
	ksceKernelDelayThreadCB(4 * 1000 * 1000);
	ksceBtStopInquiry();
#endif*/

	while (bt_thread_run) {
		int ret;
		unsigned int evf_out;

		ret = ksceKernelWaitEventFlagCB(bt_thread_evflag_uid, EVF_EXIT,
			SCE_EVENT_WAITOR | SCE_EVENT_WAITCLEAR_PAT, &evf_out, NULL);
		if (ret < 0)
			continue;

		if (evf_out & EVF_EXIT)
			break;
	}

	if (ds3_connected)
		ksceBtStartDisconnect(ds3_mac0, ds3_mac1);

	ksceBtUnregisterCallback(bt_cb_uid);

	ksceKernelDeleteCallback(bt_cb_uid);

	return 0;
}

void _start() __attribute__ ((weak, alias ("module_start")));

int module_start(SceSize argc, const void *args)
{
	int ret;
	tai_module_info_t SceBt_modinfo;

	log_reset();

	LOG("ds3vita by xerpi\n");

	SceBt_modinfo.size = sizeof(SceBt_modinfo);
	ret = taiGetModuleInfoForKernel(KERNEL_PID, "SceBt", &SceBt_modinfo);
	if (ret < 0) {
		LOG("Error finding SceBt module\n");
		goto error_find_scebt;
	}

	/* SceBt patch */
	/*unsigned short thumb_nop[] = {0x00bf};

	SceBt_sub_2292CE4_0x2292D18_patch_uid = taiInjectDataForKernel(KERNEL_PID,
		SceBt_modinfo.modid, 0, 0x2292D18 - 0x2280000, thumb_nop, sizeof(thumb_nop));

	LOG("SceBt_sub_2292CE4_0x2292D18_patch_uid: 0x%08X\n", SceBt_sub_2292CE4_0x2292D18_patch_uid);*/

	/* SceBt hooks */
	SceBt_sub_22999C8_hook_uid = taiHookFunctionOffsetForKernel(KERNEL_PID,
		&SceBt_sub_22999C8_ref, SceBt_modinfo.modid, 0,
		0x22999C8 - 0x2280000, 1, SceBt_sub_22999C8_hook_func);

	SceBt_sub_22947E4_hook_uid = taiHookFunctionOffsetForKernel(KERNEL_PID,
		&SceBt_sub_22947E4_ref, SceBt_modinfo.modid, 0,
		0x22947E4 - 0x2280000, 1, SceBt_sub_22947E4_hook_func);

	/* Patch PAD Type */
	BIND_FUNC_EXPORT_HOOK(SceCtrl_ksceCtrlGetControllerPortInfo, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0xF11D0D30);

	/* SceCtrl hooks:
	 * sceCtrlPeekBufferNegative -> ksceCtrlPeekBufferNegative
	 * sceCtrlPeekBufferNegative2 -> none
	 * sceCtrlPeekBufferPositive -> ksceCtrlPeekBufferPositive
	 * sceCtrlPeekBufferPositive2 -> none
	 * sceCtrlPeekBufferPositiveExt -> none
	 * sceCtrlPeekBufferPositiveExt2 -> none
	 * sceCtrlReadBufferNegative -> ksceCtrlReadBufferNegative
	 * sceCtrlReadBufferNegative2 -> none
	 * sceCtrlReadBufferPositive -> ksceCtrlReadBufferPositive
	 * sceCtrlReadBufferPositive2 -> none
	 * sceCtrlReadBufferPositiveExt -> none
	 * sceCtrlReadBufferPositiveExt2 -> none
	 */
	BIND_FUNC_EXPORT_HOOK(SceCtrl_ksceCtrlPeekBufferNegative, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0x19895843);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlPeekBufferNegative2, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0x81A89660);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_ksceCtrlPeekBufferPositive, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0xEA1D3A34);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlPeekBufferPositive2, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0x15F81E8C);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlPeekBufferPositiveExt, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0xA59454D3);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlPeekBufferPositiveExt2, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0x860BF292);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_ksceCtrlReadBufferNegative, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0x8D4E0DD1);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlReadBufferNegative2, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0x27A0C5FB);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_ksceCtrlReadBufferPositive, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0x9B96A1AA);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlReadBufferPositive2, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0xC4226A3E);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlReadBufferPositiveExt, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0xE2D99296);

	BIND_FUNC_EXPORT_HOOK(SceCtrl_sceCtrlReadBufferPositiveExt2, KERNEL_PID,
		"SceCtrl", TAI_ANY_LIBRARY, 0xA7178860);

	SceKernelHeapCreateOpt opt;
	opt.size = 0x1C;
	opt.uselock = 0x100;
	opt.field_8 = 0x10000;
	opt.field_C = 0;
	opt.field_10 = 0;
	opt.field_14 = 0;
	opt.field_18 = 0;

	bt_mempool_uid = ksceKernelCreateHeap("ds3vita_mempool", 0x100, &opt);
	LOG("Bluetooth mempool UID: 0x%08X\n", bt_mempool_uid);

	bt_thread_evflag_uid = ksceKernelCreateEventFlag("ds3vita_bt_thread_evflag",
							 0, 0, NULL);
	LOG("Bluetooth thread event flag UID: 0x%08X\n", bt_thread_evflag_uid);

	bt_thread_uid = ksceKernelCreateThread("ds3vita_bt_thread", ds3vita_bt_thread,
		0x3C, 0x1000, 0, 0x10000, 0);
	LOG("Bluetooth thread UID: 0x%08X\n", bt_thread_uid);
	ksceKernelStartThread(bt_thread_uid, 0, NULL);

	LOG("module_start finished successfully!\n");

	return SCE_KERNEL_START_SUCCESS;

error_find_scebt:
	return SCE_KERNEL_START_FAILED;
}

int module_stop(SceSize argc, const void *args)
{
	SceUInt timeout = 0xFFFFFFFF;

	bt_thread_run = 0;

	if (bt_thread_evflag_uid)
		ksceKernelSetEventFlag(bt_thread_evflag_uid, EVF_EXIT);

	if (bt_thread_uid > 0) {
		bt_thread_run = 0;
		ksceKernelWaitThreadEnd(bt_thread_uid, NULL, &timeout);
		ksceKernelDeleteThread(bt_thread_uid);
	}

	if (bt_thread_evflag_uid)
		ksceKernelDeleteEventFlag(bt_thread_evflag_uid);

	if (bt_mempool_uid > 0) {
		ksceKernelDeleteHeap(bt_mempool_uid);
	}

	if (SceBt_sub_22999C8_hook_uid > 0) {
		taiHookReleaseForKernel(SceBt_sub_22999C8_hook_uid,
			SceBt_sub_22999C8_ref);
	}

	if (SceBt_sub_22947E4_hook_uid > 0) {
		taiHookReleaseForKernel(SceBt_sub_22947E4_hook_uid,
			SceBt_sub_22947E4_ref);
	}

	UNBIND_FUNC_HOOK(SceCtrl_ksceCtrlGetControllerPortInfo);

	UNBIND_FUNC_HOOK(SceCtrl_ksceCtrlPeekBufferNegative);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlPeekBufferNegative2);
	UNBIND_FUNC_HOOK(SceCtrl_ksceCtrlPeekBufferPositive);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlPeekBufferPositive2);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlPeekBufferPositiveExt);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlPeekBufferPositiveExt2);
	UNBIND_FUNC_HOOK(SceCtrl_ksceCtrlReadBufferNegative);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlReadBufferNegative2);
	UNBIND_FUNC_HOOK(SceCtrl_ksceCtrlReadBufferPositive);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlReadBufferPositive2);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlReadBufferPositiveExt);
	UNBIND_FUNC_HOOK(SceCtrl_sceCtrlReadBufferPositiveExt2);

	/*if (SceBt_sub_2292CE4_0x2292D18_patch_uid > 0) {
		taiInjectReleaseForKernel(SceBt_sub_2292CE4_0x2292D18_patch_uid);
	}*/

	log_flush();

	return SCE_KERNEL_STOP_SUCCESS;
}
