#!/bin/bash
#SBATCH --mail-type=all
#SBATCH --job-name="IUAVBS"
#SBATCH --mail-user=lucas.pacheco@inf.unibe.ch
#SBATCH --mail-type=all
#SBATCH --time=2-00:00:00
#SBATCH --mem-per-cpu=16G
#SBATCH --tmp=300G
#SBATCH --partition=long
#SBATCH --cpus-per-task=4


module load Boost/1.66.0-foss-2018a
module load  Python/3.8.2-GCCcore-9.3.0
./run.sh
