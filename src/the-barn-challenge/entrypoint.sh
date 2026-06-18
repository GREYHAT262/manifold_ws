#!/bin/bash
source /manifold_ws/devel/setup.bash
cd /manifold_ws/src/the-barn-challenge
exec ${@:1}
