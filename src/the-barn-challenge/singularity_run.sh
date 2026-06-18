#!/bin/bash
singularity exec -i --nv -n --network=none -p -B `pwd`:/manifold_ws/src/the-barn-challenge ${1} /bin/bash /manifold_ws/src/the-barn-challenge/entrypoint.sh ${@:2}
