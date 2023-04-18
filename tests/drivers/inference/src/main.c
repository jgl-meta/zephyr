#include <zephyr/kernel.h>
#include <zephyr/ztest.h>

extern const struct device *get_pe_device(void);
extern const struct task_id_t *get_mock_task_id();
extern const struct fba_submission *get_mock_submission();

void run_admin_command()
{
    const struct device *pe_array_mock = get_pe_device();
    const struct task_id_t *t1 = get_mock_task_id();
    const struct fba_submission *s1 = get_mock_submission();
    struct pe_control_api* api_mock = NULL;

    if (!device_is_ready(pe_array_mock)) {
        /* Not ready, do not use */
        err = -ENODEV;
        goto done;
    }
    api_mock = (struct pe_control_api*)pe_array_mock->api;


    api_mock->submit_admin_command(*t1, s1);
}

ZTEST_SUITE(admin_command, NULL, run_admin_command, NULL, NULL, NULL);
