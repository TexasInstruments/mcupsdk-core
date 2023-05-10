SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`
SCRIPTPATH=`dirname $SCRIPTPATH`
version=`jq -r '.version' $SCRIPTPATH/.metadata/product.json`
packagedir=k3-respart-tool-$version

cd $SCRIPTPATH/
rm -rf $packagedir
mkdir $packagedir
mkdir $packagedir/modules
mkdir $packagedir/deviceData
mkdir $packagedir/data
mkdir $packagedir/out

# Create a file to describe the baseline for package
commit=`git log --oneline -1 --no-decorate`
echo > $packagedir/manifest.txt
echo "Package created on `date`" >> $packagedir/manifest.txt
echo "Baseline commit\n\t$commit" >> $packagedir/manifest.txt

# Copy the static data
cp README.md LICENSE $packagedir/
cp modules/*.js $packagedir/modules/
cp data/tifsResRem.json $packagedir/data/
cp -r .metadata scripts templates $packagedir/
rm -rf $packagedir/scripts/node_modules

# Create baseline JSON files
echo "[]" > $packagedir/data/SOC.json
jq '.devices = []' .metadata/product.json > $packagedir/.metadata/product.json

while [ $# -gt 0 ]
do
	soc=$1

	# Copy the soc specific data
	cp -r modules/$soc $packagedir/modules/
	cp -r data/$soc $packagedir/data/
	cp out/*$soc* $packagedir/out/
	cp -r deviceData/$soc $packagedir/deviceData/

	# Update baseline JSON files with all device names for a SoC
	dev_names=`jq -r '.[] | select(.shortName == "'$soc'") | .soc' data/SOC.json`
	for device in $dev_names;
	do
		echo "Adding $soc - $device"
		jq -s --tab --sort-keys '(.[0] + [.[1][] | select(.soc == "'$device'")])' \
			$packagedir/data/SOC.json data/SOC.json > temp
		mv temp $packagedir/data/SOC.json

		jq --tab --sort-keys '.devices = .devices + ["'$device'"]' \
			$packagedir/.metadata/product.json > temp
		mv temp $packagedir/.metadata/product.json
	done
	shift
done

jq --tab --sort-keys 'sort_by(.soc)' \
	$packagedir/data/SOC.json > temp
mv temp $packagedir/data/SOC.json

# Create final package
zip -qq -r $packagedir.zip $packagedir
