if [ "$#" -ne 1 ]; then
    SEED=123
else
    SEED=$1
fi
matlab -nodisplay -nosplash -nodesktop -r "seed = $SEED; run('randomArmExample.m');"
