import { canopen, output, CanMessage, Util } from 'ECB'

const SLAVE_ID = 0x0A;
const MASTER_ID = 0x01;
const sleep = (ms: number) => new Promise(resolve => setTimeout(resolve, ms));

const device: any = new canopen.Device({ id: MASTER_ID });

let heartbeatResolver: ((val: boolean) => void) | null = null;

async function waitForSlaveOnline(timeout = 10000): Promise<boolean> {
    return new Promise((resolve) => {
        heartbeatResolver = resolve;
        setTimeout(() => { 
            if (heartbeatResolver) { heartbeatResolver = null; resolve(false); } 
        }, timeout);
    });
}

async function runOptimizedProcedure() {
    console.log("--- 🚀 HPMICRO CANopen 测试 ---");

    try {
        console.log("[1] 发送远程重置指令 (nmt.resetNode)...");
        device.nmt.resetNode(SLAVE_ID); 
        
        await sleep(1000);

        if (await waitForSlaveOnline()) {
            console.log("✅ 检测到从站 Boot-up 或 Heartbeat 信号信号");
        } else {
            console.log("⚠️ 未检测到心跳事件，尝试 SDO 直接确认链路...");
            await device.sdo.upload({ deviceId: SLAVE_ID, index: 0x1000, subIndex: 0, dataType: canopen.DataType.UNSIGNED32 });
            console.log("✅ SDO 链路确认成功");
        }

        console.log("[2] 执行参数存储 (SDO download)...");
        await device.sdo.download({ 
            deviceId: SLAVE_ID, index: 0x1010, subIndex: 1, 
            data: 0x65766173, dataType: canopen.DataType.UNSIGNED32,
            timeout: 10000 
        });
        console.log("✅ 存储确认成功");

        console.log("[3] 配置从站动态 PDO 映射...");
        device.nmt.enterPreOperational(SLAVE_ID); 
        await sleep(500); 

        const U32 = canopen.DataType.UNSIGNED32;
        const U8 = canopen.DataType.UNSIGNED8;

        await device.sdo.download({ deviceId: SLAVE_ID, index: 0x1800, subIndex: 1, data: 0x8000018A, dataType: U32, timeout: 15000 });
        await device.sdo.download({ deviceId: SLAVE_ID, index: 0x1A00, subIndex: 0, data: 0, dataType: U8, timeout: 15000 });
        await device.sdo.download({ deviceId: SLAVE_ID, index: 0x1A00, subIndex: 1, data: 0x21020020, dataType: U32, timeout: 15000 });
        await device.sdo.download({ deviceId: SLAVE_ID, index: 0x1A00, subIndex: 0, data: 1, dataType: U8, timeout: 15000 });
        await device.sdo.download({ deviceId: SLAVE_ID, index: 0x1800, subIndex: 1, data: 0x0000018A, dataType: U32, timeout: 15000 });
        console.log("✅ 映射配置完成");

        console.log("[4] 启动远程节点 (nmt.startNode)...");
        device.nmt.startNode(SLAVE_ID); 
        
        for (let i = 1; i <= 3; i++) {
            await sleep(2000);
            console.log(`📥 采样 [${i}]: 0x2102 字典实时值 = ${device.getValue(0x2102)}`);
        }

        console.log("\n🏁 🎉 [SUCCESS] 测试成功！");

    } catch (e) {
        console.log("❌ 流程中断: " + (e as any).message);
    }
}

Util.Init(() => {
    device.eds.addSdoClientParameter(SLAVE_ID, 0x600 + SLAVE_ID, 0x580 + SLAVE_ID);
    device.eds.addEntry(0x2102, { parameterName: 'Counter', dataType: canopen.DataType.UNSIGNED32 });
    device.eds.addReceivePdo({ cobId: 0x180 + SLAVE_ID, transmissionType: 255, dataObjects: [device.eds.getEntry(0x2102)] });

    device.eds.addHeartbeatConsumer(SLAVE_ID, 1000); 
    
    device.nmt.on('bootup', (nodeId: number) => {
        if (nodeId === SLAVE_ID && heartbeatResolver) {
            const r = heartbeatResolver; heartbeatResolver = null; r(true);
        }
    });
    device.nmt.on('heartbeat', (nodeId: number) => {
        if (nodeId === SLAVE_ID && heartbeatResolver) {
            const r = heartbeatResolver; heartbeatResolver = null; r(true);
        }
    });

    device.addListener('message', (message: any) => {
        try {
            let finalData: Buffer;
            if (Buffer.isBuffer(message.data)) {
                finalData = message.data;
            } else if (Array.isArray(message.data)) {
                finalData = Buffer.from(message.data);
            } else if (typeof message.data === 'number') {
                finalData = Buffer.from([message.data, SLAVE_ID]);
            } else {
                finalData = Buffer.from([message.data]);
            }

            (output as any)({ id: message.id, data: finalData, dir: 'OUT', msgType: { idType: 0 } }).catch(() => {}); 
        } catch (e) { }
    });

    Util.OnCan(true, (msg: any) => {
        const data = Buffer.from(msg.data);
        if (msg.id === 0x700 + SLAVE_ID && heartbeatResolver) {
            const r = heartbeatResolver; heartbeatResolver = null; r(true);
        }
        try {
            device.receive({ id: msg.id, data: data, len: data.length });
        } catch (e) { }
    });

    device.start();
    device.nmt.startNode();
    runOptimizedProcedure();
});