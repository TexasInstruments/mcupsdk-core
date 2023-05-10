topdir=$(git rev-parse --show-toplevel)
sysfw_repo=$(
	cd $topdir/../system-firmware
	pwd
)
autogen_repo=$(
	cd $topdir/../system-firmware-autogen
	pwd
)

prettify_json() {
	file=$1
	sortkey=$2

	if [ "$sortkey" != "" ]; then
		filter="sort_by(.$sortkey)"
	else
		filter="."
	fi

	jq --tab --sort-keys "$filter" $file >pretty.json
	mv pretty.json $file
	echo "  Converted to pretty format"
}

gen_files() {
	soc=$1
	case $soc in
	j721e)
		sysfw_soc=j721e
		soc_json=$autogen_repo/src_input/csl/j7es/json/J7ES.json
		;;
	j7200)
		sysfw_soc=j7200
		soc_json=$autogen_repo/src_input/csl/j7vcl/json/J7VCL.json
		;;
	am65x)
		sysfw_soc=am6x
		soc_json=$autogen_repo/src_input/csl/maxwell/json/MAXWELL_PG2.json
		;;
	am64x)
		sysfw_soc=am64x
		soc_json=$autogen_repo/src_input/csl/am64x/json/AM64x.json
		;;
	am243x)
		sysfw_soc=am64x
		soc_json=$autogen_repo/src_input/csl/am64x/json/AM64x.json
		;;
	am62x)
		sysfw_soc=am62x
		soc_json=$autogen_repo/src_input/csl/am62dual/json/AM62dual.json
		;;
	am62a)
		sysfw_soc=am62a
		soc_json=$autogen_repo/src_input/csl/am62a/json/AM62A.json
		;;
	j721s2)
		sysfw_soc=j721s2
		soc_json=$autogen_repo/src_input/csl/j7_aep/json/J7_AEP.json
		;;
	j784s4)
		sysfw_soc=j784s4
		soc_json=$autogen_repo/src_input/csl/j7_ahp/json/J7_AHP.json
		;;
	esac

	# Parse SYSFW documentation Resource assignments
	mkdir -p $topdir/data/$soc
	node $topdir/scripts/parse_resource_rst.js --soc $soc \
		--doc $sysfw_repo/docs/public/5_soc_doc/$sysfw_soc/resasg_types.rst \
		--dep $topdir/data/$soc/ResDependencies.json
	echo "Generated data/$soc/Resources.json"
	prettify_json $topdir/data/$soc/Resources.json

	# Parse SYSFW documentation Host descriptions
	node $topdir/scripts/parse_hostname_rst.js --soc $soc \
		--doc $sysfw_repo/docs/public/5_soc_doc/$sysfw_soc/hosts.rst \
		--names $topdir/data/$soc/HostNames.json \
		--firewall $sysfw_repo/docs/public/5_soc_doc/$sysfw_soc/firewalls.rst
	echo "Generated data/$soc/Hosts.json"
	prettify_json $topdir/data/$soc/Hosts.json hostId

	# Parse SoC json to generate QoS and firewall data
	if [ -f $soc_json ]; then
		node $topdir/scripts/parse_soc_qos.js --soc $soc --doc $soc_json
		echo "Generated data/$soc/Qos.json"
		prettify_json $topdir/data/$soc/Qos.json deviceName

		node $topdir/scripts/parse_soc_firewalls.js --soc $soc --doc $soc_json \
			--dname $topdir/data/$soc/DeviceName.json \
			--firewall $sysfw_repo/docs/public/5_soc_doc/$sysfw_soc/firewalls.rst
		echo "Generated data/$soc/Firewall.json"
		prettify_json $topdir/data/$soc/Firewall.json name
	fi

	# Generate host modules
	mkdir -p $topdir/modules/$soc
	node $topdir/scripts/gen_host_modules.js --soc $soc
	echo "Generated modules/$soc/XXX.syscfg.js"
	echo
}

gen_initial_user_json() {
	soc=$1

	# Copy original Firewall.json to generate the DeviceName.json
	jq --tab --sort-keys "[.[] | {name: .name, protected_inst: .protected_inst}]" \
		$topdir/data/$soc/Firewall.json > $topdir/data/$soc/DeviceName.json

	# Copy original Resource.json to generate the ResDependencies.json
	jq --tab --sort-keys "[.[] | {deviceName: .deviceName, subtypeName: .subtypeName, utype: .utype}]" \
		$topdir/data/$soc/Resources.json > $topdir/data/$soc/ResDependencies.json

	# Copy original Hosts.json to generate the HostNames.json
	jq --tab --sort-keys "[.[] | {hostName: .hostName, displayName: \"FILL\"}]" \
		$topdir/data/$soc/Hosts.json > $topdir/data/$soc/HostNames.json
}

if [ "$1" = "" ]; then
	echo "Specify which soc to build"
	exit
elif [ "$1" = "all" ]; then
	gen_files j721e
	gen_files j7200
	gen_files am65x
	gen_files am64x
	gen_files am62x
	gen_files am62a
	gen_files j721s2
	gen_files j784s4
else
	gen_files $1
fi
