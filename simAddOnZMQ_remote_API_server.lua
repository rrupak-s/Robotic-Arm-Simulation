sim=require'sim'
removeLazyLoaders()

zmqRemoteApi={}

function zmqRemoteApi.verbose()
    return sim.getNamedInt32Param('zmqRemoteApi.verbose') or 0
end

function zmqRemoteApi.require(name)
    _G[name]=require(name)
end

function zmqRemoteApi.info(obj)
    if type(obj)=='string' then obj=zmqRemoteApi.getField(obj) end
    if type(obj)~='table' then return obj end
    local ret={}
    for k,v in pairs(obj) do
        if type(v)=='table' then
            ret[k]=zmqRemoteApi.info(v)
        elseif type(v)=='function' then
            ret[k]={func={}}
        elseif type(v)~='function' then
            ret[k]={const=v}
        end
    end
    return ret
end

function zmqRemoteApi.getField(f)
    local v=_G
    for w in string.gmatch(f,'[%w_]+') do
        v=v[w]
        if not v then return nil end
    end
    return v
end

function zmqRemoteApi.handleRequest(req)
    if zmqRemoteApi.verbose()>1 then
        print('Received request:',req)
    end
    local resp={}
    if req['func']~=nil and req['func']~='' then
        local func=zmqRemoteApi.getField(req['func'])
        local args=req['args'] or {}
        if not func then
            resp['error']='No such function: '..req['func']
        else
            local status,retvals=pcall(function()
                local ret={func(unpack(args))}
                return ret
            end)
            resp[status and 'ret' or 'error']=retvals
        end
    elseif req['eval']~=nil and req['eval']~='' then
        local status,retvals=pcall(function()
            -- cannot prefix 'return ' here, otherwise non-trivial code breaks
            local ret={loadstring(req['eval'])()}
            return ret
        end)
        resp[status and 'ret' or 'error']=retvals
    end
    resp['success']=resp['error']==nil
    if zmqRemoteApi.verbose()>1 then
        print('Sending response:',resp)
    end
    return resp
end

function zmqRemoteApi.handleRawMessage(rawReq)
    local status,req=pcall(cbor.decode,rawReq)
    if status then
        local resp=zmqRemoteApi.handleRequest(req)
        local status,resp=pcall(cbor.encode,resp)
        if status then return resp end
        return cbor.encode({success=false,error=resp})
    else
        sim.addLog(sim.verbosity_errors,'Decode error: '..req)
        return ''
    end
end

function zmqRemoteApi.handleQueue()
    function dumpBytes(x)
        if sim.getNamedStringParam('zmqRemoteApi.debugBinaryFormat')=='base64' then
            return 'base64='..sim.transformBuffer(req,sim.buffer_uint8,0,0,sim.buffer_base64)
        else
            return table.join(map(partial(string.format,'%02X'),string.bytes(x)),' ')
        end
    end

    local t=sim.getSystemTime()
    while true do
        local rc,revents=simZMQ.poll({rpcSocket},{simZMQ.POLLIN},0)
        if rc<=0 then break end

        local rc,req=simZMQ.recv(rpcSocket,0)

        if zmqRemoteApi.verbose()>2 then
            print('Received raw request: (len='..#req..') '..dumpBytes(req))
        end

        local resp=zmqRemoteApi.handleRawMessage(req)

        if zmqRemoteApi.verbose()>2 then
            print('Sending raw response: (len='..#resp..') '..dumpBytes(resp))
        end

        simZMQ.send(rpcSocket,resp,0)
        if sim.getSystemTime()-t>maxTimeSlot then break end
    end
end

function zmqRemoteApi.publishStepCount()
    if zmqRemoteApi.verbose()>3 then
        print('Publishing simulationTimeStepCount='..simulationTimeStepCount)
    end
    simZMQ.send(cntSocket,sim.packUInt32Table{simulationTimeStepCount},0)
end

function sysCall_info()
    return {autoStart=sim.getNamedBoolParam('zmqRemoteApi.autoStart')~=false,menu='Connectivity\nZMQ remote API server'}
end

function sysCall_init()
    simZMQ=require'simZMQ'
    simZMQ.__raiseErrors(true) -- so we don't need to check retval with every call
    rpcPort=sim.getNamedInt32Param('zmqRemoteApi.rpcPort') or 23000
    cntPort=sim.getNamedInt32Param('zmqRemoteApi.cntPort') or (rpcPort+1)
    maxTimeSlot=sim.getNamedFloatParam('zmqRemoteApi.maxTimeSlot') or 0.005
    if zmqRemoteApi.verbose()>0 then
        sim.addLog(sim.verbosity_scriptinfos,string.format('ZeroMQ Remote API server starting (rpcPort=%d, cntPort=%d)...',rpcPort,cntPort))
    end
    -- cbor=require 'cbor' -- encodes strings as buffers, always. DO NOT USE!!
    cbor=require'org.conman.cbor'
    context=simZMQ.ctx_new()
    rpcSocket=simZMQ.socket(context,simZMQ.REP)
    simZMQ.bind(rpcSocket,string.format('tcp://*:%d',rpcPort))
    cntSocket=simZMQ.socket(context,simZMQ.PUB)
    simZMQ.setsockopt(cntSocket,simZMQ.CONFLATE,sim.packUInt32Table{1})
    simZMQ.bind(cntSocket,string.format('tcp://*:%d',cntPort))
    if zmqRemoteApi.verbose()>0 then
        sim.addLog(sim.verbosity_scriptinfos,'ZeroMQ Remote API server started')
    end
    simulationTimeStepCount=0
    steppingClients={}
    steppedClients={}
end

function sysCall_cleanup()
    if not simZMQ then return end
    simZMQ.close(cntSocket)
    simZMQ.close(rpcSocket)
    simZMQ.ctx_term(context)
    if zmqRemoteApi.verbose()>0 then
        sim.addLog(sim.verbosity_scriptinfos,'ZeroMQ Remote API server stopped')
    end
end

function sysCall_addOnScriptSuspend()
    return {cmd='cleanup'}
end

function sysCall_addOnScriptSuspended()
    return {cmd='cleanup'}
end

function sysCall_nonSimulation()
    zmqRemoteApi.handleQueue()
end

function sysCall_suspended()
    zmqRemoteApi.handleQueue()
end

function sysCall_realTimeIdle()
    zmqRemoteApi.handleQueue()
end

function sysCall_beforeMainScript()
    zmqRemoteApi.handleQueue()
    local outData
    if next(steppingClients)~=nil then
        local canStep=true
        for uuid,v in pairs(steppingClients) do
            if steppedClients[uuid]==nil then
                canStep=false
                break
            end
        end
        outData={doNotRunMainScript=(not canStep)}
    end
    return outData
end

function sysCall_beforeSimulation()
    simulationTimeStepCount=0
    zmqRemoteApi.publishStepCount()
end

function sysCall_actuation()
    steppedClients={}
    simulationTimeStepCount=simulationTimeStepCount+1
    zmqRemoteApi.publishStepCount()
end

function sysCall_afterSimulation()
    zmqRemoteApi.publishStepCount() -- so that the last client.step(True) doesn't block
    steppingClients={}
    steppedClients={}
end

function setStepping(enable,uuid)
    if uuid==nil then
        uuid='ANY' -- to support older clients
    end
    if enable then
        steppingClients[uuid]=true
    else
        steppingClients[uuid]=nil
    end
    steppedClients[uuid]=nil
end

function step(uuid)
    if uuid==nil then
        uuid='ANY' -- to support older clients
    end
    steppedClients[uuid]=true
end

-- via the remote API, we should always return a string:
_S.readCustomDataBlock=sim.readCustomDataBlock
function sim.readCustomDataBlock(obj,tag)
    local retVal=_S.readCustomDataBlock(obj,tag)
    if retVal==nil then
        retVal=''
    end
    return retVal
end
