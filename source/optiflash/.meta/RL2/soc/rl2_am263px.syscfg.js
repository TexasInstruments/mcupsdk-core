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
    rl2:
    {
        max_instance: function () {return 1;},
        validate_src_range_start_address: function (address)
        {
            return check_alignment(address, 4096);
        },
        validate_src_range_end_address: function(address)
        {
            return check_alignment(address, 4096);
        },
        validate_remote_address: function(a)
        {
            return check_alignment(a, 4096);
        },
        validate_remote_size: function(sz)
        {
            if(sz < 4096 && sz !== 0)
            {
                return resp(false, "should not be more than 4096");
            }

            return check_alignment(sz, 64);
        }
    }
}