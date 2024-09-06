
// Application size and secure boot time values 

//For AM263x
const am263x_mcelf_signed_appsizes = [64, 128, 256, 512, 768, 1024, 1536];
const am263x_mcelf_signed_boottime = [40.5313, 41.0514, 45.6229, 60.4598, 75.3044, 90.1629, 116.1281];


const am263x_mcelf_encrypted_appsizes = [64, 128, 256, 512, 768, 1024, 1536];
const am263x_mcelf_encrypted_boottime = [42.4021, 44.5898, 56.1803, 76.3964, 96.6124, 116.9272, 157.2603];

//For AM263Px

//28.7848 + 0.00813173 x
const am263px_mcelf_signed_appsizes = [64, 128, 256, 512, 768, 1024, 1536, 2048, 2560];
const am263px_mcelf_signed_boottime = [29.3522, 29.8321, 30.7448, 32.6115, 35.0746, 37.1117, 41.5637, 46.3405, 48.772];

// y = 28.8668 + 0.0336063 x
const am263px_mcelf_encrypted_appsizes = [64, 128, 256, 512, 768, 1024, 1536, 2048, 2560];
const am263px_mcelf_encrypted_boottime = [31.2624, 33.3115, 37.4878, 45.8581, 53.6344, 64.1311, 80.4861, 97.6925, 114.8989];


// Information Table

const am263x_software_table = ` Software Specifications
<table>
<tr><td>MCU+ SDK version</td><td>v10.00.00</td></tr>  
<tr><td>TIFSMCU version</td><td>v10.00.00</td></tr>  
<tr><td>Application Image Format version</td><td>MCELF</td></tr>  
<tr><td>SBL Size</td><td>50KB</td></tr>  
<tr><td>SBL Properties</td><td>Signed + Encrypted</td></tr>  
<tr><td>HSM Runtime Size</td><td>70KB</td></tr>  
<tr><td>HSM Runtime Properties</td><td>Signed + Encrypted</td></tr>  
<tr><td>Maximum Segment of Application</td><td>64KB</td></tr> 
</table>
`;

const am263px_software_table = ` Software Specifications
<table>
<tr><td>MCU+ SDK version</td><td>v10.00.00</td></tr>  
<tr><td>TIFSMCU version</td><td>v10.00.00</td></tr>  
<tr><td>Application Image Format version</td><td>MCELF</td></tr>  
<tr><td>SBL Size</td><td>60KB</td></tr>
<tr><td>SBL Properties</td><td>Signed + Encrypted</td></tr>    
<tr><td>HSM Runtime Size</td><td>61KB</td></tr>
<tr><td>HSM Runtime Properties</td><td>Signed + Encrypted</td></tr>    
<tr><td>Maximum Segment of Application</td><td>64KB</td></tr> 
</table>
`;

const am263x_hardware_table = ` <br>
Hardware Specifications
<table>
<tr><td>Application Core Frequency</td><td>400 MHz</td></tr>  
<tr><td>Flash Frequency</td><td>80 MHz</td></tr>
<tr><td>Flash Properties</td><td>Quad SPI Flash Interface with SDR</td></tr>
<tr><td>HSM Frequency </td><td>200 MHz</td></tr>  
</table>
`;

const am263px_hardware_table = ` <br>
Hardware Specifications
<table>
<tr><td>Application Core Frequency</td><td>400 MHz</td></tr>  
<tr><td>Flash Frequency</td><td>133 MHz</td></tr>
<tr><td>Flash Properties</td><td>Octal SPI Flash Interface with DDR support</td></tr>
<tr><td>HSM Frequency </td><td>200 MHz</td></tr>  
</table>
`;

function onSubmitInfo() {
    xValues1 = [];
    yValues1 = [];

    xValues2 = [];
    yValues2 = [];

    if (document.getElementById("mySOC").value === "am263px") {
        console.log(document.getElementById("mySOC").value)
        xValues1 = am263px_mcelf_signed_appsizes;
        yValues1 = am263px_mcelf_signed_boottime;

        xValues2 = am263px_mcelf_encrypted_appsizes;
        yValues2 = am263px_mcelf_encrypted_boottime;

        document.getElementById('table-container').innerHTML = am263px_software_table + am263px_hardware_table;
    }
    else {
        xValues1 = am263x_mcelf_signed_appsizes;
        yValues1 = am263x_mcelf_signed_boottime;

        xValues2 = am263x_mcelf_encrypted_appsizes;
        yValues2 = am263x_mcelf_encrypted_boottime;

        document.getElementById('table-container').innerHTML = am263x_software_table + am263x_hardware_table;
    }

    new Chart("myChart", {
        type: "line",
        data: {
            labels: xValues1,
            datasets: [{
                label: "Authenticated Boot",
                fill: false,
                lineTension: 0,
                backgroundColor: "rgba(28, 6, 160, 0.8)",
                borderColor: "rgba(28, 6, 160, 0.8)",
                data: yValues1
            },
            {
                label: "Authenticated + Decrypted Boot",
                fill: false,
                lineTension: 0,
                backgroundColor: "rgba(18, 96, 45, 0.8)",
                borderColor: "rgba(18, 96, 45, 0.8)",
                data: yValues2
            }
            ]
        },
        options: {
            legend: {
                display: true
            },
            scales: {
                xAxes: [
                    {
                        scaleLabel: {
                            display: true,
                            labelString: 'Application Size in KB'
                        }
                    }
                ],
                yAxes: [
                    {
                        scaleLabel: {
                            display: true,
                            labelString: 'Boot time in milliseconds'
                        }
                    }],
            },
            plugins: {
                title: {
                    display: true,
                    text: 'Custom Chart Title',
                    position: 'bottom',
                    padding: {
                        top: 10,
                        bottom: 30
                    }
                }
            }
        }
    });
}
