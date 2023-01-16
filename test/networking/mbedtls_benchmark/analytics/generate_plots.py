import pandas as pd
import matplotlib.pyplot as plt

f = open("results.txt", "r")

# Creating an empty dictionary
myDict = {}

x_axis = list()

for lines in f:
    if("Current Data size" in lines):
        str_list = lines.split('B')
        str_list = str_list[0].split(':')
        lines = str_list[1].replace(" ","")

        x_axis.append(int(lines))

    if("KiB/s" in lines):
        str_list = lines.split('cycles/byte')
        str_list = str_list[0].split('KiB/s,')
        cycle_count = str_list[1].replace(" ","")

        str_list = str_list[0].split(':')
        throughput = str_list[1].replace(" ","")

        key_value = str_list[0].replace(" ","")

        try:
            myDict[key_value+"_thpt"].append(int(throughput))
        except:
            myDict[key_value+"_thpt"] = list()
            myDict[key_value+"_thpt"].append(int(throughput))

        try:
            myDict[key_value+"_cyclecount"].append(int(cycle_count))
        except:
            myDict[key_value+"_cyclecount"] = list()
            myDict[key_value+"_cyclecount"].append(int(cycle_count))

    elif("handshake/s" in lines):
        str_list = lines.split('handshake/s')
        str_list = str_list[0].split(':')
        throughput = str_list[1].replace(" ","")
        key_value = str_list[0].replace(" ","")

        try:
            myDict[key_value+"_thpt"].append(int(throughput))
        except:
            myDict[key_value+"_thpt"] = list()
            myDict[key_value+"_thpt"].append(int(throughput))

    elif("public/s" in lines):
        str_list = lines.split('public/s')
        str_list = str_list[0].split(':')
        throughput = str_list[1].replace(" ","")
        key_value = str_list[0].replace(" ","")

        try:
            myDict[key_value+"_thpt"].append(int(throughput))
        except:
            myDict[key_value+"_thpt"] = list()
            myDict[key_value+"_thpt"].append(int(throughput))

    elif("private/s" in lines):
        str_list = lines.split('private/s')
        str_list = str_list[0].split(':')
        throughput = str_list[1].replace(" ","")
        key_value = str_list[0].replace(" ","")

        try:
            myDict[key_value+"_thpt"].append(int(throughput))
        except:
            myDict[key_value+"_thpt"] = list()
            myDict[key_value+"_thpt"].append(int(throughput))

    elif("sign/s" in lines):
        str_list = lines.split('sign/s')
        str_list = str_list[0].split(':')
        throughput = str_list[1].replace(" ","")
        key_value = str_list[0].replace(" ","")

        try:
            myDict[key_value+"_thpt"].append(int(throughput))
        except:
            myDict[key_value+"_thpt"] = list()
            myDict[key_value+"_thpt"].append(int(throughput))

    elif("verify/s" in lines):
        str_list = lines.split('verify/s')
        str_list = str_list[0].split(':')
        throughput = str_list[1].replace(" ","")
        key_value = str_list[0].replace(" ","")

        try:
            myDict[key_value+"_thpt"].append(int(throughput))
        except:
            myDict[key_value+"_thpt"] = list()
            myDict[key_value+"_thpt"].append(int(throughput))

f.close()

for key in myDict:
    fig, ax = plt.subplots()
    str_list = key.split('_')
    plt.title(str_list[0])
    plt.xlabel('Size of Data')
    plt.ylabel((str_list[1].upper()))

    tempDict = {}
    tempDict['Size of Data'] = x_axis
    tempDict[(str_list[1].upper())] = myDict[key]

    s = pd.DataFrame(tempDict)
    s.plot(kind = 'line', x = 'Size of Data', y = (str_list[1].upper()))
    ax.plot(x_axis, myDict[key])
    fig.savefig("plots/"+key+'_plot.png')
    plt.close(fig)

    with pd.ExcelWriter("output.xlsx", mode='a', if_sheet_exists='replace', engine="openpyxl") as writer:
        s.to_excel(writer, sheet_name=str_list[0])
