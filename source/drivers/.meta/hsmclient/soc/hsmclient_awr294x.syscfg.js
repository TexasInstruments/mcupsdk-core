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
    else if(selfCore == "c66ss0")
    {
        return "c66ss0";
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
