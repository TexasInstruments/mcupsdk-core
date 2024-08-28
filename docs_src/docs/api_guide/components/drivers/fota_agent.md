# FOTA Agent {#DRIVERS_FOTA_AGENT_PAGE}

[TOC]

The Fota Agent driver provides API to Update Firmware over The Air 
by scheduling the writes in chunks of smaller size while the new image 
is being recieved over a certain protocol(CAN for example).

## Features Supported

- Supports Writes in 4KB chunks
- Only ELF image format supported
- Both XIP and Non-XIP files

## Features NOT Supported

- RPRC image format

## Example Usage

Include the below file to access the APIs
\snippet Fota_agent_sample.c include

Init API
\snippet Fota_agent_sample.c fota_init

Write Start API
\snippet Fota_agent_sample.c fota_write_start

Write Update API
\snippet Fota_agent_sample.c fota_write_update

Write End API
\snippet Fota_agent_sample.c fota_write_end

## API

\ref DRV_FOTA_AGENT_MODULE