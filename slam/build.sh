BUILD_FODER="build"

mkdir $BUILD_FODER
rm -rf $BUILD_FODER/*
cd $BUILD_FODER
cmake .. -B. -GNinja
ninja

cd ..