#
# windows script to add x509 certificate to binary/ELF
param
(
	[string]$ELF,
	[string]$BIN,
	[string]$KEY,
	[string]$OUTPUT = $null,
	[string]$LOADADDR = '0x00040000',
	[ValidateSet('R5','DMSC_I', 'DMSC_O')][string]$CERT_SIGN = 'DMSC_I',
	[ValidateSet('EFUSE_DEFAULT','SPLIT_MODE')][string]$MODE_R5 = 'EFUSE_DEFAULT',
	[int]$FIRMWARE_BOARD_CONFIG=0
)

#paths
$SCRIPT_DIR = split-path -parent $MyInvocation.MyCommand.Definition

if ( $BIN.length -eq  0 ) {
		$BASE_PATH = $(split-path -Path $ELF)
		$WORK_FOLDER = [io.path]::GetFileNameWithoutExtension($ELF)
} else {
		$BASE_PATH = $(split-path -Path $BIN)
		$WORK_FOLDER = [io.path]::GetFileNameWithoutExtension($BIN)
}

#create working dir from input file
#to allow parallel makes
$WORK_DIR="$BASE_PATH\$WORK_FOLDER"
New-Item -ItemType Directory -Force -Path $WORK_DIR

#variables
$SHA = 'sha512'
$TEMP_X509="$WORK_DIR\x509-temp.cert"
$CERT="$WORK_DIR\$CERT_SIGN" + '-cert.bin'
$RAND_KEY="$WORK_DIR\eckey.pem"
$VALID_CERT_SIGNS=@('R5', 'DMSC_I', 'DMSC_O')
$OBJCOPY="$Env:TOOLCHAIN_PATH_A53" + '\bin\aarch64-none-elf-objcopy'
$X509_TEMPLATE="$SCRIPT_DIR\x509template.txt"
$VALID_R5_BOOTCORE_OPTS=@("EFUSE_DEFAULT", "SPLIT MODE")

# setup default output path if not specified
if ( $OUTPUT.length -eq  0 ) {
	$OUTPUT="$WORK_DIR\x509-firmware.bin"
}

#check if openssl is present
Write-Host "Checking for OpenSSL..."
try { Invoke-Expression "openssl version" }
catch { Write-Host "Not found! Please install OpenSSL"
    exit 1
}

$sha_oids = @{}
$sha_oids.sha256='2.16.840.1.101.3.4.2.1'
$sha_oids.sha384='2.16.840.1.101.3.4.2.2'
$sha_oids.sha512='2.16.840.1.101.3.4.2.3'
$sha_oids.sha224='2.16.840.1.101.3.4.2.4'

function gen_key(){
	Write-Host "Generating random key..."
	Invoke-Expression "openssl ecparam -out $RAND_KEY -name prime256v1 -genkey"
}

$options_help = @{}
function usage() {

	if ( $args.count -ne 0 ) {
		Write-Host "ERROR: $args"
	}

	Write-Host -NoNewline "Usage: $(split-path $MyInvocation.PSCommandPath -Leaf) "
	foreach($option in $options_help.Keys) {
		$param_short_desc = $options_help.$option.split(':')[0]
		Write-Host -NoNewline "[-$option $param_short_desc] "
	}
	Write-Host ""
	Write-Host "Where:"
	foreach($option in $options_help.Keys) {
		$param_short_desc, $param_long_desc = $options_help.$option.split(':')
		Write-Host "   -$option $param_short_desc$param_long_desc"
	}
	Write-Host 'Examples of usage:-'
	Write-Host '# Generate x509 certificate with random key from elf'
	Write-Host "	 $(split-path $MyInvocation.PSCommandPath -Leaf) -e ti-sci-firmware-am65xx.elf -o dmsc.bin -l 0x40000"
	Write-Host '# Generate x509 certificate with random key from bin'
	Write-Host "	$(split-path $MyInvocation.PSCommandPath -Leaf) -b ti-sci-firmware-am65xx.bin -o dmsc.bin -l 0x40000"
}

$options_help.e="elf_file:`t`tELF file that needs to be signed"
$options_help.b="bin_file:`t`ttBin file that needs to be signed"
$options_help.k="key_file:`t`tFile with key inside it. If not provided script generates a random key."
$options_help.o="output_file:`tName of the final output file. The default is x509-firmware.bin"
$options_help.l="loadaddr:`t`tTarget load address of the binary in hex. Default to $LOADADDR"
$options_help.f="FIRMWARE_BOARD_CONFIG:Option to indicate this is a board configuration value. Supported values (0-1). Default value $FIRMWARE_BOARD_CONFIG"

if ( ( $BIN.length -eq  0 ) -and ( $ELF.length -eq  0 ) ) {
	usage "Either Input bin file or ELF file to sign"
	exit 1
}

#Generate random key if user doesn't provide a key.
if ( $KEY.length -eq 0 ) {
	gen_key
	$KEY=$RAND_KEY
}

if ( "$CERT_SIGN" -eq 'R5' ) {
	$BOOTCORE_ID=16
	$CERT_TYPE=1
	if ( "$MODE_R5" -eq 'SPLIT_MODE' ) {
		$BOOTCORE_OPTS=2
	} else {
		$BOOTCORE_OPTS=0
	}
} else {
	$BOOTCORE_ID=0
	$BOOTCORE_OPTS=0
	if ( "$CERT_SIGN" -eq 'DMSC_O' ) {
		$CERT_TYPE=3
	} else {
		$CERT_TYPE=2
	}
}

if ( $BIN.length -eq  0 ) {
	Write-Host "Generating bin from elf $ELF"
	$BIN="$WORK_DIR\firmware.bin"
	trap {
		Write-Host "ERROR: Generating bin from $ELF failed. OBJCOPY?"
		exit 1
	}
	Invoke-Expression "$(OBJCOPY) -g -S --gap-fill 0x0 -O binary $ELF $BIN"
}

$SHA_OID=$sha_oids.$SHA
$SHA_VAL = $(Invoke-Expression "openssl dgst -$SHA -hex $BIN") -replace '.*= ', ''
$BIN_SIZE=(Get-Item $BIN).length
$ADDR = ([Int32]::Parse($LOADADDR.split('x')[1], 'HexNumber')).ToString('X8')

function gen_cert() {
	if ( $FIRMWARE_BOARD_CONFIG -eq 0 ) {
		Write-Host "$CERT_SIGN Certificate being generated :"
		Write-Host "`tX509_CFG = $TEMP_X509"
		Write-Host "`tKEY = $KEY"
		Write-Host "`tBIN = $BIN"
		Write-Host "`tCERT TYPE = $CERT_SIGN, $CERT_TYPE"
		Write-Host "`tCORE ID = $BOOTCORE_ID"
		Write-Host "`tLOADADDR = 0x$ADDR"
		Write-Host "`tIMAGE_SIZE = $BIN_SIZE"
		Write-Host "`tBOOT_OPTIONS = $BOOTCORE_OPTS"
		Write-Host "`tSHA OID  is $SHA_OID"
		Write-Host "`tSHA  is $SHA_VAL"
	}
	else
	{
		$X509_TEMPLATE="$SCRIPT_DIR\x509template_boardcfg.txt"
	}
	$SSL_CONF_FILE = (Get-Content -Raw $X509_TEMPLATE) | ForEach-Object {
				$_.replace("TEST_IMAGE_LENGTH", "$BIN_SIZE").
				replace("TEST_IMAGE_SHA_OID", "$SHA_OID").
				replace("TEST_IMAGE_SHA_VAL", "$SHA_VAL").
				replace("TEST_CERT_TYPE", "$CERT_TYPE").
				replace("TEST_BOOT_CORE_ID", "$BOOTCORE_ID").
				replace("TEST_BOOT_CORE_OPTS", "$BOOTCORE_OPTS").
				replace("TEST_BOOT_ADDR", "$ADDR")
				}
	[IO.File]::WriteAllText($TEMP_X509, $SSL_CONF_FILE)
	Invoke-Expression "openssl req -new -x509 -key $KEY -nodes -outform DER -out $CERT -config $TEMP_X509 -$SHA"
}

gen_cert
Get-Content $CERT, $BIN -Enc Byte -Read 512 | Set-Content $OUTPUT  -Enc Byte

if ( $FIRMWARE_BOARD_CONFIG -eq 0 ) {
	Write-Host "SUCCESS: Image $OUTPUT generated. Good to boot"
}
else
{
	Write-Host "SUCCESS: Image $OUTPUT generated."
}

#Remove all intermediate files
Remove-Item $TEMP_X509
Remove-Item $CERT
if ( $KEY -eq $RAND_KEY ) {
	Remove-Item $RAND_KEY

}
if ( $BIN -eq "$WORK_DIR\firmware.bin" ) {
	Remove-Item $BIN

}
if ( $OUTPUT -ne "$WORK_DIR\x509-firmware.bin" ) {
	Remove-Item $WORK_DIR -Recurse
}
