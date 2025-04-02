import os
import json
import glob
from pathlib import Path

#function to check and rewrite cfg file depending on rtos tag given in enabled sections
def revise_cfg_file(cfg_file_path, file_list_path, rtos_docs_enable_flag):
	#read cfg file content
	with open(Path(cfg_file_path), "r") as cfg_f:
		cfg_file_content = cfg_f.readlines()

	revised_cfg_content = []
	#if rtos_docs_enable_flag is disabled, given file entries will not be present in the cfg files.
	if(not rtos_docs_enable_flag):
		file_name_not_found = 1
		for line in cfg_file_content:
			for file in file_list_path:
				if file in line:
					file_name_not_found = 0
					break
				else:
					file_name_not_found = 1
			if (file_name_not_found == 1):
				revised_cfg_content.append(line)

		with open(Path(cfg_file_path), "w") as cfg_f:
			cfg_f.writelines(revised_cfg_content)
	else:
		#if rtos_docs_enable_flag is enabled, given file entries will be present in the cfg files
		file_name_found = 0
		file_not_found_list = []

		for file in file_list_path:
			for line in cfg_file_content:
				if file in line:
					file_name_found = 1
					break
				else:
					file_name_found = 0

			if (file_name_found == 0):
				file_not_found_list.append(file)

		revised_cfg_content = cfg_file_content

		for file in file_not_found_list:
			for root,dirs,files in os.walk(os.getcwd()):
				if file in files:
					line = os.path.join(root,file)
					break
			index = line.find("docs_src")
			line = line[index : ]
			line = "INPUT+= $(MCU_PLUS_SDK_PATH)/"+str(line)+"\n"
			revised_cfg_content.append(line)

		with open(Path(cfg_file_path), "w") as cfg_f:
			cfg_f.writelines(revised_cfg_content)

if __name__ == "__main__":
	# Open and read the JSON file
	with open('rtos_doc_config.json', 'r') as json_file:
		j_dict = json.load(json_file)

	for device in j_dict["device"]:
		for os_name in j_dict["device"][str(device)]:
			os_key_dict = j_dict["device"][str(device)][str(os_name)]
			enabled_sections_os_tag = os_key_dict["enabled_sections_tag"]
			include_cfg_fname = os_key_dict["includes"]["cfg"]
			include_file_list = os_key_dict["includes"]["files"]
			examples_cfg_file_name = os_key_dict["examples"]["cfg"]
			example_md_list = os_key_dict["examples"]["files"]
			component_list = os_key_dict["components"]

			includes_cfg_file_path = os.path.abspath("./device/"+str(device)+"/"+str(include_cfg_fname))
			with open(includes_cfg_file_path, "r") as file_includes_cfg_read:
				lines = file_includes_cfg_read.readlines()
			rtos_docs_enable_flag = 0
			for line in lines:
				#check if Rtos tag is present in includes.cfg
				if "ENABLED_SECTIONS" in line and str(enabled_sections_os_tag) in line:
					rtos_docs_enable_flag = 1

      	    #revise includes.cfg
			revise_cfg_file(includes_cfg_file_path, include_file_list, rtos_docs_enable_flag)

        	#revise examples.cfg
			examples_cfg_file_path = os.path.abspath("./device/"+str(device)+"/"+str(examples_cfg_file_name))
			revise_cfg_file(examples_cfg_file_path, example_md_list, rtos_docs_enable_flag)

			#revise component cfg files
			for component_key in component_list.keys():
				component_file_list_path = component_list[component_key]["files"]
				component_cfg_file = component_list[component_key]["cfg"]
				component_cfg_file_path = 0
				for root,dirs,files in os.walk(os.getcwd()):
					if component_cfg_file in files:
						component_cfg_file_path = os.path.join(root,component_cfg_file)
						break
				revise_cfg_file(component_cfg_file_path, component_file_list_path, rtos_docs_enable_flag)
