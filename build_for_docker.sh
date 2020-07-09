# Perform a regular build, and do so in a new "build" directory
mkdir -p build
( cd build ;  cmake .. ;  make -j 8 )


# (Optional) Build the simple tests too
( cd test/test_millipede ; cmake . ; make -j 8 )


# Copy the relevant files to the docker directory
echo -n "Copying ./build ... "
cp -r ./build/  ./docker/
echo "done"
echo -n "Copying ./test ... "
cp -r ./test/   ./docker/
echo "done"
echo -n "Copying ./data ... "
cp -r ./data/   ./docker/
echo "done"

