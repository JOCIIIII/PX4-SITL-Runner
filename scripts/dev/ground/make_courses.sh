#!/bin/bash
# The course CSVs (ground_sim/courses) of the UGV / USV review and the planned-path missions (Hybrid PP on the
# layouts, ground_pp.sh) - what ground_batch.py flies. Run after setup.sh.
set -e
HERE=$(dirname $(readlink -f "$0"))
GS=~/Documents/A4VAI-SITL/ground_sim
C=$GS/courses; mkdir -p $C $GS/pp
gc() { python3 $HERE/ground_courses.py "$@"; }
gc line --size 20 --spacing 0.5 --out $C/ugv_line.csv
gc circle --size 4 --spacing 0.3 --out $C/ugv_circle.csv
gc square --size 10 --corner 2 --spacing 0.5 --out $C/ugv_square_r2.csv
gc square --size 10 --spacing 0.5 --out $C/ugv_square_sharp.csv
gc slalom --size 24 --corner 2 --spacing 0.4 --out $C/ugv_slalom.csv
gc line --size 20 --spacing 0.5 --start-offset 3,60 --out $C/ugv_line_offset.csv
gc line --size 80 --spacing 2 --out $C/usv_line.csv
gc circle --size 15 --spacing 1.0 --out $C/usv_circle.csv
gc square --size 50 --corner 12 --spacing 2 --out $C/usv_square_r12.csv
gc slalom --size 120 --corner 8 --spacing 2 --out $C/usv_slalom.csv
gc line --size 80 --spacing 2 --start-offset 10,60 --out $C/usv_line_offset.csv
$HERE/ground_pp.sh ugv $HERE/layouts/ugv_yard.json $GS/pp/ugv_yard
$HERE/ground_pp.sh usv $HERE/layouts/usv_harbour.json $GS/pp/usv_harbour
cp $GS/pp/ugv_yard/mission.csv $C/pp_ugv_yard.csv
cp $GS/pp/usv_harbour/mission.csv $C/pp_usv_harbour.csv
python3 $HERE/ground_world.py $HERE/layouts/ugv_yard.json ugv $GS/worlds/ugv_yard.world
python3 $HERE/ground_world.py $HERE/layouts/usv_harbour.json usv $GS/worlds/usv_harbour.world
