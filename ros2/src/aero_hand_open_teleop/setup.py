# Copyright 2025 TetherIA, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from glob import glob
from setuptools import find_packages, setup

package_name = "aero_hand_open_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/manus_teleop.launch.py"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=['setuptools', 'numpy', 'mediapipe', 'opencv-python'],
    zip_safe=True,
    maintainer="mohit",
    maintainer_email="mohityadav@tetheria.ai",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        'console_scripts': [
            'manus_joint_states_retargeting = aero_hand_open_teleop.manus_joint_states_retargeting:main',
            'mediapipe_mocap = aero_hand_open_teleop.mediapipe_mocap:main',
            'mediapipe_retargeting = aero_hand_open_teleop.mediapipe_retargeting:main',
        ],
    },
)
