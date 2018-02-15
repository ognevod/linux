/*
 * ELVEES VPOUT Controller DRM Driver
 *
 * Copyright 2017 RnD Center "ELVEES", JSC
 *
 * Based on tilcdc:
 * Copyright (C) 2012 Texas Instruments
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/component.h>

#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "vpout-drm-drv.h"
#include "vpout-drm-external.h"
#include "vpout-drm-panel.h"

static LIST_HEAD(module_list);

void vpout_drm_module_init(struct vpout_drm_module *mod, const char *name,
			   const struct vpout_drm_module_ops *funcs)
{
	mod->name = name;
	mod->funcs = funcs;
	INIT_LIST_HEAD(&mod->list);
	list_add(&mod->list, &module_list);
}

void vpout_drm_module_cleanup(struct vpout_drm_module *mod)
{
	list_del(&mod->list);
}

static struct drm_framebuffer *vpout_drm_fb_create(struct drm_device *dev,
		struct drm_file *file_priv, struct drm_mode_fb_cmd2 *mode_cmd)
{
	return drm_fb_cma_create(dev, file_priv, mode_cmd);
}

static void vpout_drm_fb_output_poll_changed(struct drm_device *dev)
{
	struct vpout_drm_private *priv = dev->dev_private;

	drm_fbdev_cma_hotplug_event(priv->fbdev);
}

static const struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = vpout_drm_fb_create,
	.output_poll_changed = vpout_drm_fb_output_poll_changed,
};

static int modeset_init(struct drm_device *dev)
{
	struct vpout_drm_private *priv = dev->dev_private;
	struct vpout_drm_module *mod;

	drm_mode_config_init(dev);

	priv->crtc = vpout_drm_crtc_create(dev);

	list_for_each_entry(mod, &module_list, list) {
		mod->funcs->modeset_init(mod, dev);
	}

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;
	dev->mode_config.max_width = 2048;
	dev->mode_config.max_height = 2048;
	dev->mode_config.funcs = &mode_config_funcs;

	return 0;
}

/*
 * DRM operations:
 */

static int vpout_drm_load(struct drm_device *dev, unsigned long flags)
{
	struct platform_device *pdev = dev->platformdev;
	struct vpout_drm_private *priv;
	struct vpout_drm_module *mod;
	struct resource *res;
	u32 bpp = 0;
	int ret;

	priv = devm_kzalloc(dev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->wq = alloc_ordered_workqueue("elvees-vpout-drm", 0);
	if (!priv->wq)
		return -ENOMEM;

	dev->dev_private = priv;

	priv->is_componentized =
		vpout_drm_get_external_components(dev->dev, NULL) > 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->mmio = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->mmio)) {
		dev_err(dev->dev, "failed to ioremap\n");
		ret = PTR_ERR(priv->mmio);
		goto fail_free_wq;
	}

	priv->clk = devm_clk_get(&pdev->dev, 0);
	if (IS_ERR(priv->clk)) {
		dev_err(dev->dev, "failed to get clock\n");
		ret = PTR_ERR(priv->clk);
		goto fail_free_wq;
	}

	ret = clk_prepare_enable(priv->clk);
	if (ret < 0) {
		dev_err(dev->dev, "failed to enable clock\n");
		goto fail_free_wq;
	}

	ret = modeset_init(dev);
	if (ret < 0) {
		dev_err(dev->dev, "failed to initialize mode setting\n");
		goto fail_disable_clk;
	}

	platform_set_drvdata(pdev, dev);

	if (priv->is_componentized) {
		ret = component_bind_all(dev->dev, dev);
		if (ret < 0)
			goto fail_mode_config_cleanup;

		ret = vpout_drm_add_external_encoders(dev, &bpp);
		if (ret < 0)
			goto fail_component_cleanup;
	}

	if ((priv->num_encoders == 0) || (priv->num_connectors == 0)) {
		dev_err(dev->dev, "no encoders/connectors found\n");
		ret = -ENXIO;
		goto fail_external_cleanup;
	}

	ret = drm_vblank_init(dev, 1);
	if (ret < 0) {
		dev_err(dev->dev, "failed to initialize vblank\n");
		goto fail_external_cleanup;
	}

	ret = drm_irq_install(dev, platform_get_irq(dev->platformdev, 0));
	if (ret < 0) {
		dev_err(dev->dev, "failed to install IRQ handler\n");
		goto fail_vblank_cleanup;
	}

	list_for_each_entry(mod, &module_list, list) {
		bpp = mod->preferred_bpp;
		if (bpp > 0)
			break;
	}

	priv->fbdev = drm_fbdev_cma_init(dev, bpp, dev->mode_config.num_crtc,
					 dev->mode_config.num_connector);
	if (IS_ERR(priv->fbdev)) {
		ret = PTR_ERR(priv->fbdev);
		goto fail_irq_uninstall;
	}

	drm_kms_helper_poll_init(dev);

	return 0;

fail_irq_uninstall:
	drm_irq_uninstall(dev);

fail_vblank_cleanup:
	drm_vblank_cleanup(dev);

fail_mode_config_cleanup:
	drm_mode_config_cleanup(dev);

fail_component_cleanup:
	if (priv->is_componentized)
		component_unbind_all(dev->dev, dev);

fail_external_cleanup:
	vpout_drm_remove_external_encoders(dev);

fail_disable_clk:
	clk_disable_unprepare(priv->clk);

fail_free_wq:
	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);

	dev->dev_private = NULL;

	return ret;
}

static int vpout_drm_unload(struct drm_device *dev)
{
	struct vpout_drm_private *priv = dev->dev_private;

	vpout_drm_remove_external_encoders(dev);

	drm_fbdev_cma_fini(priv->fbdev);
	drm_kms_helper_poll_fini(dev);
	drm_mode_config_cleanup(dev);
	drm_vblank_cleanup(dev);

	drm_irq_uninstall(dev);

	flush_workqueue(priv->wq);
	destroy_workqueue(priv->wq);

	dev->dev_private = NULL;

	return 0;
}

static void vpout_drm_lastclose(struct drm_device *dev)
{
	struct vpout_drm_private *priv = dev->dev_private;

	drm_fbdev_cma_restore_mode(priv->fbdev);
}

static irqreturn_t vpout_drm_irq(int irq, void *arg)
{
	struct drm_device *dev = arg;
	struct vpout_drm_private *priv = dev->dev_private;

	return vpout_drm_crtc_irq(priv->crtc);
}

static int vpout_drm_enable_vblank(struct drm_device *dev, unsigned int pipe)
{
	return 0;
}

static void vpout_drm_disable_vblank(struct drm_device *dev, unsigned int pipe)
{
}

static const struct file_operations fops = {
	.owner              = THIS_MODULE,
	.open               = drm_open,
	.release            = drm_release,
	.unlocked_ioctl     = drm_ioctl,
	.poll               = drm_poll,
	.read               = drm_read,
	.llseek             = no_llseek,
	.mmap               = drm_gem_cma_mmap,
};

static struct drm_driver vpout_drm_driver = {
	.driver_features    = DRIVER_HAVE_IRQ | DRIVER_GEM | DRIVER_MODESET,
	.load               = vpout_drm_load,
	.unload             = vpout_drm_unload,
	.lastclose          = vpout_drm_lastclose,
	.set_busid          = drm_platform_set_busid,
	.irq_handler        = vpout_drm_irq,
	.get_vblank_counter = drm_vblank_no_hw_counter,
	.enable_vblank      = vpout_drm_enable_vblank,
	.disable_vblank     = vpout_drm_disable_vblank,
	.gem_free_object    = drm_gem_cma_free_object,
	.gem_vm_ops         = &drm_gem_cma_vm_ops,
	.dumb_create        = drm_gem_cma_dumb_create,
	.dumb_map_offset    = drm_gem_cma_dumb_map_offset,
	.dumb_destroy       = drm_gem_dumb_destroy,
	.fops               = &fops,
	.name               = "vpout-drm",
	.desc               = "ELVEES VPOUT Controller DRM",
	.date               = "20170825",
	.major              = 1,
	.minor              = 0,
};

/*
 * Platform driver:
 */

static int vpout_drm_bind(struct device *dev)
{
	return drm_platform_init(&vpout_drm_driver, to_platform_device(dev));
}

static void vpout_drm_unbind(struct device *dev)
{
	drm_put_dev(dev_get_drvdata(dev));
}

static const struct component_master_ops vpout_drm_comp_ops = {
	.bind = vpout_drm_bind,
	.unbind = vpout_drm_unbind,
};

static int vpout_drm_pdev_probe(struct platform_device *pdev)
{
	struct component_match *match = NULL;
	int ret;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "device-tree data is missing\n");
		return -ENXIO;
	}

	ret = vpout_drm_get_external_components(&pdev->dev, &match);
	if (ret < 0)
		return ret;

	if (ret == 0)
		return drm_platform_init(&vpout_drm_driver, pdev);

	return component_master_add_with_match(&pdev->dev,
					       &vpout_drm_comp_ops, match);
}

static int vpout_drm_pdev_remove(struct platform_device *pdev)
{
	struct drm_device *ddev = dev_get_drvdata(&pdev->dev);
	struct vpout_drm_private *priv = ddev->dev_private;

	if (!priv)
		return 0;

	if (priv->is_componentized)
		component_master_del(&pdev->dev, &vpout_drm_comp_ops);
	else
		drm_put_dev(platform_get_drvdata(pdev));

	return 0;
}

static const struct of_device_id vpout_drm_of_match[] = {
	{ .compatible = "elvees,mcom02-vpout", },
	{ },
};
MODULE_DEVICE_TABLE(of, vpout_drm_of_match);

static struct platform_driver vpout_drm_platform_driver = {
	.probe = vpout_drm_pdev_probe,
	.remove = vpout_drm_pdev_remove,
	.driver = {
		.name = "vpout-drm",
		.of_match_table = vpout_drm_of_match,
	},
};

static int __init vpout_drm_drm_init(void)
{
	vpout_drm_panel_init();

	return platform_driver_register(&vpout_drm_platform_driver);
}

static void __exit vpout_drm_drm_fini(void)
{
	platform_driver_unregister(&vpout_drm_platform_driver);

	vpout_drm_panel_fini();
}

module_init(vpout_drm_drm_init);
module_exit(vpout_drm_drm_fini);

MODULE_AUTHOR("Alexey Kiselev <akiselev@elvees.com");
MODULE_DESCRIPTION("ELVEES VPOUT Controller DRM Driver");
MODULE_LICENSE("GPL");
