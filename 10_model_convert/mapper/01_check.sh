#!/usr/bin/env sh
# Copyright (c) 2020 Horizon Robotics.All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of Horizon Robotics Inc. This is proprietary information owned by
# Horizon Robotics Inc. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of Horizon Robotics Inc.

cd $(dirname $0) || exit
set -e
model_type="onnx"
caffe_model="/open_explorer/model_convert/line_tracking.onnx"
march="bernoulli2"

hb_mapper checker --model-type ${model_type} \
                  --model ${caffe_model} \
                  --march ${march}
