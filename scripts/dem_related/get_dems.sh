#!/bin/bash

echo "Getting ready to download DEMs..."

dems=$(cat scripts/dem_related/dems_list.txt)

echo "Downloading $(echo $dems | wc -w) DEMs..."
echo "Using 8 parallel processes..."
echo "This may take a while..."

CWD=$(pwd)
mkdir -p tmp
cd tmp

cat $CWD/scripts/dem_related/dems_list.txt | xargs -n 1 -P 8 wget -nv -nc

cd $CWD

echo "Finished downloading DEMs."
echo "Check the tmp/ directory for the downloaded files."

echo "If all is well, please run the next script: ./scripts/dem_related/extract_dems.sh"
echo "This script will extract the DEMs and prepare them for use in the simulation."
