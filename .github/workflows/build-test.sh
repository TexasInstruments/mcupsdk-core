#
#  Copyright (C) 2025 Texas Instruments Incorporated.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#


#########################################################################
#                               Params                                  #
#########################################################################
#Get user input
while [ $# -gt 0 ]; do
  case "$1" in
    --device=*)
      device="${1#*=}"
      ;;
    --profile_list=*)
      profile_list="${1#*=}"
      ;;
    --enable_test_build=*)
      enable_test_build="${1#*=}"
      ;;
    -h|--help)
    echo Usage: $0 [options]
    echo
    echo Options:
    echo --device                   Device to be built
    echo --profile_list             Profile list separated by comma to be built. Default to release,debug
    echo --enable_test_build        Enable test build
    exit 0
      ;;
    *)
      printf "Error: Invalid argument $1!!\n"
  esac
  shift
done

#Default value if not provided
: ${device:="am64x"}
: ${enable_test_build:="true"}
: ${profile_list:="debug,release"}

#Convert , to space
profile_list=`echo ${profile_list} | sed -e "s|\,| |g"`

#########################################################################
#                               Log files                               #
#########################################################################
log_dir=$GITHUB_WORKSPACE/logs
build_dir=$GITHUB_WORKSPACE/mcu_plus_sdk
pr_checkout_dir=$GITHUB_WORKSPACE/pr_checkout
build_log=${log_dir}/build.log
build_error_log=${log_dir}/build_error.log


 #########################################################################
 #                               Functions                               #
 #########################################################################
print_time_diff() {
    local end_time=$(date +%s)
    local start_time=$1
    local deltatime=$(( end_time-start_time ))
    local hours=$(( deltatime/3600 ))
    local minutes=$(( deltatime/60 ))
    local minutes=$(( minutes%60 ))
    local seconds=$(( deltatime%60 ))
    printf "$2: %d:%02d:%02d\n" $hours $minutes $seconds
    printf " \n"
}

make_folders() {
    echo "Making required folder..."
    mkdir -p ${log_dir}
    echo "Making required folder ... Done"
    echo " "
}

proc=`nproc`
repo_init() {
    mkdir workarea 
    cd workaread
    echo "Doing repo init and sync ..."
    local start_time=`date +%s`
    sudo apt-get update
    sudo apt-get -y install repo
    
    repo init -u https://github.com/TexasInstruments/mcupsdk-manifests.git -m ${device}/dev.xml -b main  --depth=1
    repo sync -j${proc} -q

    #Show the current branch/git status
    repo forall -c "pwd;git branch -vv | cut -d ' ' -f 1-4"

    print_time_diff $start_time "Repo Init Time"
    echo "Doing repo init and sync ... Done"
    echo " "

    #Checkout the PR
    pushd ${build_dir} 1>/dev/null 
    
    git fetch origin pull/${PR_NUMBER}/head:pr_${PR_NUMBER} -q
    git switch pr_${PR_NUMBER}
    git log --oneline -n5
    popd 1>/dev/null

}

download_components() {
    echo "Downloading Components ..."

    local start_time=`date +%s`

    mkdir ${HOME}/ti

    find ./mcupsdk_setup -name "*.sh" -execdir chmod u+x {} +
    ./mcupsdk_setup/${device}/download_components.sh
    pip3 install pyserial xmodem tqdm pyelftools construct

    print_time_diff $start_time "Download Components Time"
    echo "Downloading Components ... Done"
    echo " "
}

check_logs() {
    if [ -e ${build_error_log} ]; then
        if [ -s ${build_error_log} ]; then
            echo "Build for SOC $device failed ...."
            echo 
            echo
            cat ${build_error_log}
            exit 1
        fi
    fi
}

build_sdk() {
    echo "Build SDK ..."
    local start_time=`date +%s`

    pushd ${build_dir} 1>/dev/null

    #Scrub build files

    echo "    Scrub Build Files for DEVICE:${device} ..."
    make -s ${jobs_option} scrub DEVICE=${device} 1>>${build_log} 2>>${build_error_log}
    make -s gen-buildfiles-clean DEVICE=${device} 1>>${build_log} 2>>${build_error_log}
    echo "    Scrub Build Files for DEVICE:${device} completed!!"
    check_logs

    echo "    Generate Build Files for DEVICE:${device} ..."
    make -s gen-buildfiles DEVICE=${device} 1>>${build_log} 2>>${build_error_log}
    echo "    Generate Build Files for DEVICE:${device} completed!!"
    check_logs

    for profile in ${profile_list}
    do
        echo "    Building for DEVICE:${device} PROFILE:${profile} ..."
        make -s -j${proc} all DEVICE=${device} PROFILE=${profile} 1>>${build_log} 2>>${build_error_log}
        echo "    Building for DEVICE:${device} PROFILE:${profile} completed!!"
        check_logs
    done

    if [ "${enable_test_build}" == "true" ]; then
        for profile in ${profile_list}
        do
            echo "    Building Tests for DEVICE:${device} PROFILE:${profile} ..."
            make -s -j${proc} tests DEVICE=${device} PROFILE=${profile} 1>>${build_log} 2>>${build_error_log}
            echo "    Building Tests for DEVICE:${device} PROFILE:${profile} completed!!"
            check_logs
        done
    fi

    echo "    Generate docs for DEVICE:${device} ..."
    make -s docs DEVICE=${device} 1>>${build_log} 2>>${build_error_log}
    echo "    Generate docs for DEVICE:${device} completed!!"
    #Redirect doxy error
    cat docs_src/docs/api_guide/doxy_warnings_${device}.txt >> ${build_error_log}
    check_logs

    popd 1>/dev/null

    print_time_diff $start_time "Build Time"
    echo "Build SDK ... Done"
}

#########################################################################
#                           Script run                                  #
#########################################################################
make_folders
repo_init
download_components
build_sdk