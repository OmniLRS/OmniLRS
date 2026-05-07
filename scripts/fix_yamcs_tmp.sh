#!/usr/bin/env bash
set -e
sudo rm -rf /tmp/images_apxs \
            /tmp/images_depth \
            /tmp/images_lander \
            /tmp/images_monitoring \
            /tmp/images_oncommand \
            /tmp/images_streaming
mkdir -p /tmp/images_apxs \
         /tmp/images_depth \
         /tmp/images_lander \
         /tmp/images_monitoring \
         /tmp/images_oncommand \
         /tmp/images_streaming
