let common = system.getScript("/common");

function get_optiflash_max_instances()
{
    return 1;
}

function get_rl2_max_instances()
{
    return 1;
}

function resp(status, msg)
{
    return {
        status: status,
        msg: msg
    };
}

function check_alignment(address, align)
{
    address = parseInt(address);
    if(address > ((2**32)-1))
    {
        return resp(false, "Address should be 32bit wide");
    }
    if(address % align)
    {
        return resp(false, "Address should be 4K aligned");
    }
    else
    {
        return resp(true, "");
    }
}

exports = {
    flc:
    {
        max_instance: function() {return 4;},
        validate_source_address: function(address)
        {
            return check_alignment(address, 4096);
        },
        validate_destination_address: function(address)
        {
            return check_alignment(address, 4096);
        },
        validate_copy_size: function(size)
        {
            return check_alignment(size, 4096);
        }
    }
}