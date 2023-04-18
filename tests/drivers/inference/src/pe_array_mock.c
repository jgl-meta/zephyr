
#include <zephyr/inference/pe.h>
#include <zephyr/inference/pe_array_mock.h>

static void exe_pe_define_extra(task_id_t tid, struct fba_submission *cur)
{
    return;
}

static void exe_pe_define(task_id_t tid, struct fba_submission *cur)
{
    return;
}

static void exe_pe_update(task_id_t tid, struct fba_submission *cur)
{
    return;
}

static void exe_pe_start(task_id_t tid, struct fba_submission *cur)
{
    return;
}

static void exe_pe_dma_tstamp(task_id_t tid, struct fba_submission *cur)
{
    return;
}

static void disable_pe_keepalive(void)
{
    return;
}

static void enable_pe_keepalive(void)
{
    return;
}

static void packet_task_complete(task_id_t task, uint16_t result)
{
    return;
}

static bool exeq_collective_handler(task_id_t tid, struct fba_submission *sub)
{
    return true;
}

static void exeq_default_handler(task_id_t tid, struct fba_submission *sub)
{
    return;
}

static void host_context_create(task_id_t tid, struct fba_submission *sub)
{
    return;
}

static void host_context_delete_begin(task_id_t tid, struct fba_submission *sub)
{
    return;
}

static void host_context_delete_end(task_id_t tid, struct fba_submission *sub)
{
    return;
}

static void time_set(task_id_t tid, struct fba_submission *sub)
{
    return;
}

static void fbia_exeq_handler_mock(task_id_t tid, struct fba_submission *sub)
{
  switch (sub->header.opcode) {
  case EXE_DEFINE_JOB:
    exe_pe_define(tid, sub);
    break;

  case EXE_DEFINE_JOB_EXTRA:
    exe_pe_define_extra(tid, sub);
    break;

  case EXE_UPDATE_JOB:
    exe_pe_update(tid, sub);
    break;

  case EXE_START_JOB:
    exe_pe_start(tid, sub);
    break;

  case EXE_DMA_TSTAMP_JOB:
    exe_pe_dma_tstamp(tid, sub);
    break;

  case EXE_DIS_PE_KEEPALIVE:
    disable_pe_keepalive();
    packet_task_complete(tid, CS_OK);
    break;

  case EXE_EN_PE_KEEPALIVE:
    enable_pe_keepalive();
    packet_task_complete(tid, CS_OK);
    break;

  default:
#ifdef CONFIG_MTIA_ENABLE_COLLECTIVES
    if (exeq_collective_handler(tid, sub)) {
      break;
    }
#endif
    exeq_default_handler(tid, sub);
    break;
  }
}

static void adminq_default_handler_mock(task_id_t tid, struct fba_submission *sub) {
  switch (sub->header.opcode) {
  case ADMIN_NOOP:
    packet_task_complete(tid, admin_noop(sub));
    break;
  case ADMIN_CREATE_CONTEXT:
    host_context_create(tid, sub);
    break;
  case ADMIN_DELETE_CONTEXT_BEGIN:
    host_context_delete_begin(tid, sub);
    break;
  case ADMIN_DELETE_CONTEXT_END:
    host_context_delete_end(tid, sub);
    break;
  case ADMIN_GET_INFO_WORDS:
    packet_task_complete(tid, admin_get_info_words(sub));
    break;
  case ADMIN_GET_RESOURCES:
    packet_task_complete(tid, admin_get_resources(sub));
    break;

#ifdef CONFIG_MON_ENABLE
  case ADMIN_MON_READ_ITEMS:
    mon_items_to_host(tid, sub);
    break;

  case ADMIN_MON_READ_GROUPS:
    mon_groups_to_host(tid, sub);
    break;

  case ADMIN_MON_READ_DATA:
    mon_cur_data_to_host(tid, sub);
    break;

  case ADMIN_MON_WRITE_DATA:
    mon_set_data_from_host(tid, sub);
    break;
#endif

  case ADMIN_TIME_SET:
    time_set(tid, sub);
    break;

  default:
    LOG_ERR("SQ %d, packet %d: invalid OPCODE (%x) in adminq",
      tid.sqid, tid.packet, sub->header.opcode);
    packet_task_complete(tid, CS_BAD_OPCODE);
  }
}

 static void fbia_sail_activate_mock(void)
 {
    return;
 }

static void fbia_sail_deactivate_mock(void)
{
    return;
}

const struct device *get_pe_device(void)
{
	const struct device *const dev = device_get_binding("pe_array_0");

	if (!device_is_ready(dev)) {
		printk("Mock PE device is not ready\n");
		return NULL;
	}

	return dev;
}

const struct task_id_t *t1 = get_mock_task_id()
{
    return NULL;
}

const struct fba_submission *s1 = get_mock_submission()
{
    return NULL;
}

static const struct pe_control_api pe_control_mock = {
    .submit_admin_command = adminq_default_handler_mock,
    .submit_work_command = fbia_exeq_handler_mock,
    .sail_activate = fbia_sail_activate_mock,
    .sail_deactivate = fbia_sail_deactivate_mock,
};

static const struct pe_control_mock_data {

};

static const struct pe_control_mock_config {

};

static int32_t mock_init(const struct device *dev) {

};

#define DEFINE_PE_MOCK(inst)                                                                       \
	static struct pe_control_mock_data pe_control_mock_data_##inst = {};                           \
	static const struct pe_control_mock_config pe_control_mock_config_##inst = {                   \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, mock_init, NULL, &pe_control_mock_data_##inst,          \
			      &pe_control_mock_config_##inst, POST_KERNEL,                           \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &pe_control_mock)

DT_INST_FOREACH_STATUS_OKAY(DEFINE_PE_MOCK);
