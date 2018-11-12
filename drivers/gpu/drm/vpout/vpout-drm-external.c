/*
 * ELVEES VPOUT Controller DRM Driver
 *
 * Copyright 2017-2018 RnD Center "ELVEES", JSC
 *
 * Based on tilcdc:
 * Copyright (C) 2015 Texas Instruments
 * Author: Jyri Sarha <jsarha@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/component.h>
#include <linux/of_graph.h>

#include "vpout-drm-drv.h"
#include "vpout-drm-external.h"

#include "vpout-drm-link.h"

static int vpout_drm_external_mode_valid(struct drm_connector *connector,
					 struct drm_display_mode *mode)
{
	struct vpout_drm_private *priv = connector->dev->dev_private;
	int ret, i;

	ret = vpout_drm_crtc_mode_valid(priv->crtc, mode);
	if (ret != MODE_OK)
		return ret;

	/* forward call to appropriate connector */
	for (i = 0; i < priv->num_slaves; i++)
		if (priv->slaves[i].connector == connector)
			break;

	if (priv->slaves[i].funcs->mode_valid)
		return priv->slaves[i].funcs->mode_valid(connector, mode);

	return MODE_OK;
}

static int
insert_proxy(struct drm_device *drm_dev, struct drm_connector *connector)
{
	struct drm_connector_helper_funcs *proxy_funcs;

	const struct drm_connector_helper_funcs *conn_funcs;
	struct vpout_drm_private *priv = drm_dev->dev_private;
	struct device *dev = drm_dev->dev;
	int idx = priv->num_slaves;

	conn_funcs = connector->helper_private;

	proxy_funcs = devm_kzalloc(dev, sizeof(*proxy_funcs), GFP_KERNEL);
	if (!proxy_funcs)
		return -ENOMEM;

	/* save original object for restore in further */
	priv->slaves[idx].funcs = conn_funcs;
	priv->slaves[idx].connector = connector;

	*proxy_funcs = *conn_funcs;

	/*
	 * insert our proxy function.
	 * This allow check drm pipeline.
	 * In latest kernel release this is integrated into the kernel.
	 */
	proxy_funcs->mode_valid = vpout_drm_external_mode_valid;

	drm_connector_helper_add(connector, proxy_funcs);
	priv->num_slaves++;

	dev_dbg(dev, "External encoder '%s' connected\n",
		connector->encoder->name);

	return 0;
}

int vpout_drm_add_external_encoders(struct drm_device *drm_dev)
{
	struct drm_connector *connector;
	int ret;

	mutex_lock(&drm_dev->mode_config.mutex);
	drm_for_each_connector(connector, drm_dev) {
		ret = insert_proxy(drm_dev, connector);
		if (ret) {
			mutex_unlock(&drm_dev->mode_config.mutex);
			return ret;
		}
	}
	mutex_unlock(&drm_dev->mode_config.mutex);

	/* link encoders with corresponding remote endpoints */
	return vpout_drm_link_encoders(drm_dev);
}

void vpout_drm_remove_external_encoders(struct drm_device *drm_dev)
{
	struct vpout_drm_private *priv = drm_dev->dev_private;
	int i;

	/* restore original objects */
	for (i = 0; i < priv->num_slaves; i++)
		drm_connector_helper_add(priv->slaves[i].connector,
					 priv->slaves[i].funcs);

	/* destroy links between encoders and endpoints */
	vpout_drm_unlink_all();
}

static int dev_match_of(struct device *dev, void *data)
{
	bool matched = dev->of_node == data;

	if (matched)
		vpout_drm_link_endpoint(dev);

	return matched;
}

int vpout_drm_get_external_components(struct device *dev,
				      struct component_match **match)
{
	struct device_node *ep = NULL;
	int count = 0;

	while ((ep = of_graph_get_next_endpoint(dev->of_node, ep))) {
		struct device_node *node;

		node = of_graph_get_remote_port_parent(ep);
		if (!node && !of_device_is_available(node)) {
			of_node_put(node);
			continue;
		}

		dev_dbg(dev, "Subdevice node '%s' found\n", node->name);
		if (match)
			component_match_add(dev, match, dev_match_of, node);
		of_node_put(node);
		count++;
	}

	return count;
}
