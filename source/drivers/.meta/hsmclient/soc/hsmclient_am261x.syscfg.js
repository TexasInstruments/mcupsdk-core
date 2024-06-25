let common = system.getScript("/common");

function getSelfCoreId(){
	let selfCore = common.getSelfSysCfgCoreName(); 
    if(selfCore == "r5fss0-0")
    {
        return "r5fss0_0";
    }
    else if(selfCore == "r5fss0-1")
    {
        return "r5fss0_1";
    }
    else if(selfCore == "r5fss1-0")
    {
        return "r5fss1_0";
    }
    else if(selfCore == "r5fss1-1")
    {
        return "r5fss1_1";
    }
    else
    {
        return "hsm0_0";
    }
}

exports =
{   
    getSelfCoreId,
}
