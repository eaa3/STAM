## Scenario 1

cd S01_INPUT
windows_thing=$(find . -name *.ini)
rm $windows_thing
echo "deleting $windows_thing"

wget http://ypcex.naist.jp/trakmark/tracking-competition/wp-content/uploads/2015/07/S01L03_VGA.zip
unzip -o S01L03_VGA.zip

wget http://ypcex.naist.jp/trakmark/tracking-competition/wp-content/uploads/2015/07/intrinsics.zip
unzip -o intrinsics.zip
mv intrinsics.xml intrinsicsS01.xml

wget http://ypcex.naist.jp/trakmark/tracking-competition/wp-content/uploads/2015/07/S01L03_VGA_patch.zip
unzip -o S01L03_VGA_patch.zip

wget http://ypcex.naist.jp/trakmark/tracking-competition/wp-content/uploads/2015/07/S01_3Ddata.csv



chmod -R a+rw .

rm S01L03_VGA.zip
rm S01L03_VGA_patch.zip
rm intrinsics.zip

cd -

## Scenario 2

cd S02_INPUT
windows_thing=$(find . -name *.ini)
rm $windows_thing
echo "deleting $windows_thing"

wget http://ypcex.naist.jp/trakmark/tracking-competition/wp-content/uploads/2015/08/S02L03_VGA.zip
unzip -o S02L03_VGA.zip

wget http://ypcex.naist.jp/trakmark/tracking-competition/wp-content/uploads/2015/08/intrinsics.zip
unzip -o intrinsics.zip
mv intrinsics.xml intrinsicsS02.xml

wget http://ypcex.naist.jp/trakmark/tracking-competition/wp-content/uploads/2015/08/S02L03_patch.zip
unzip -o S02L03_VGA_patch.zip

wget http://ypcex.naist.jp/trakmark/tracking-competition/wp-content/uploads/2015/08/S02_3Ddata.csv

chmod -R a+rw .

rm S02L03_VGA.zip
rm S02L03_VGA_patch.zip
rm intrinsics.zip

cd -


## Scenario 3

cd S03_INPUT
windows_thing=$(find . -name *.ini)
rm $windows_thing
echo "deleting $windows_thing"

wget http://ypcex.naist.jp/trakmark/tracking-competition/wp-content/uploads/2015/08/S03L03_VGA.zip
unzip -o S03L03_VGA.zip

wget http://ypcex.naist.jp/trakmark/tracking-competition/wp-content/uploads/2015/07/intrinsics.zip
unzip -o intrinsics.zip
mv intrinsics.xml intrinsicsS03.xml

wget http://ypcex.naist.jp/trakmark/tracking-competition/wp-content/uploads/2015/08/S03L03_VGA_patch.zip
unzip -o S03L03_VGA_patch.zip

wget http://ypcex.naist.jp/trakmark/tracking-competition/wp-content/uploads/2015/08/S03_3Ddata.csv

chmod -R a+rw .

rm S03L03_VGA.zip
rm S03L03_VGA_patch.zip
rm intrinsics.zip

cd -

# Creating symbolic links to build dir

source setup_symb_links.bash


