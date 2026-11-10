import { canopen, output, CanMessage, Util } from 'ECB'
import * as fs from 'fs'
import * as path from 'path'

function getScriptDir(): string {
    try {
        const map = JSON.parse(fs.readFileSync(__filename + '.map', 'utf8'));
        const src = (map.sources as string[]).filter(s => s.endsWith('.ts') && !s.includes('webpack:')).pop();
        if (src) return path.dirname(path.resolve(__dirname, src));
    } catch (e) { }
    return __dirname;
}

const BIN_FILE_PATH = path.join(getScriptDir(), 'test_image', 'test_image_hpm6750evk2.bin');
const SLAVE_ID = 0x0A;
const MASTER_ID = 0x01;
const sleep = (ms: number) => new Promise(resolve => setTimeout(resolve, ms));

const device: any = new canopen.Device({ id: MASTER_ID });

let heartbeatResolver: ((val: boolean) => void) | null = null;
let isRebooting = false; 

function trimMcubootImage(buf: Buffer): Buffer {
    const MCUBOOT_MAGIC = 0x96F3B83D;
    const TLV_INFO_MAGIC = 0x6907;
    const TLV_PROT_INFO_MAGIC = 0x6908;
    try {
        if (buf.length < 32 || buf.readUInt32LE(0) !== MCUBOOT_MAGIC) {
            return buf;
        }
        const hdrSize = buf.readUInt16LE(8);
        const protTlvSize = buf.readUInt16LE(10);
        const imgSize = buf.readUInt32LE(12);
        let end = hdrSize + imgSize + protTlvSize;
        if (end + 4 <= buf.length) {
            const tlvMagic = buf.readUInt16LE(end);
            if (tlvMagic === TLV_INFO_MAGIC || tlvMagic === TLV_PROT_INFO_MAGIC) {
                end += buf.readUInt16LE(end + 2);
            }
        }
        if (end > 32 && end < buf.length) {
            console.log(`✂️  裁剪掉尾部填充: ${buf.length} -> ${end} 字节 (省 ${buf.length - end} 字节)`);
            return buf.subarray(0, end);
        }
    } catch (e) { }
    return buf;
}

function loadFirmwareBuffer(): Buffer | null {
    try {
        if (!fs.existsSync(BIN_FILE_PATH)) {
            console.log("❌ 找不到固件文件: " + BIN_FILE_PATH);
            return null;
        }
        const raw = fs.readFileSync(BIN_FILE_PATH);
        const text = raw.toString('utf8').trim();
        const buf = trimMcubootImage(/^[0-9a-fA-F\s]+$/.test(text) && text.replace(/\s/g, '').length % 2 === 0
            ? Buffer.from(text.replace(/\s/g, ''), 'hex')
            : raw);
        console.log(`✅ 固件加载成功，实际大小: ${buf.length} 字节`);
        return buf;
    } catch (e) {
        console.log(`❌ 文件读取异常: ${e}`);
        return null;
    }
}

async function waitForHeartbeat(timeout = 30000): Promise<boolean> {
    console.log(`⏳ 正在等待 0x${SLAVE_ID.toString(16)} 的上线信号...`);
    return new Promise((resolve) => {
        heartbeatResolver = resolve;
        setTimeout(() => { if (heartbeatResolver) { heartbeatResolver = null; resolve(false); } }, timeout);
    });
}

async function runOtaProcedure() {
    console.log("--- 🚀 HPMICRO CANopen OTA TEST ---");

    // 首先加载固件
    const firmware = loadFirmwareBuffer();
    if (!firmware) {
        console.log("❌ 固件加载失败，流程终止");
        return;
    }

    try {
        console.log("\n[1] 切换从站至 Pre-Operational...");
        device.nmt.enterPreOperational(SLAVE_ID); 
        await sleep(1000);

        console.log("\n[2] 开始 OTA 下载流程...");
        await device.sdo.download({ deviceId: SLAVE_ID, index: 0x1F51, subIndex: 1, data: 0, dataType: canopen.DataType.UNSIGNED8, timeout: 10000 });
        await device.sdo.download({ deviceId: SLAVE_ID, index: 0x1F51, subIndex: 1, data: 3, dataType: canopen.DataType.UNSIGNED8, timeout: 30000 });
        
        console.log("🚀 注入镜像 (分段传输中)...");

        await device.sdo.download({ deviceId: SLAVE_ID, index: 0x1F50, subIndex: 1, data: firmware, timeout: 60000 });

        console.log("-> [核心] 发送触发重启指令 (1F51:01 = 0x01)...");
        const rebootPromise = device.sdo.download({
            deviceId: SLAVE_ID, index: 0x1F51, subIndex: 1,
            data: 1, dataType: canopen.DataType.UNSIGNED8, timeout: 2000 
        });

        await sleep(50);
        isRebooting = true;
        await rebootPromise.catch(() => {});

        console.log("🤫 等待重启...");
        await sleep(5000);
        isRebooting = false;

        if (await waitForHeartbeat()) {
            console.log("📡 设备已上线...");
            await sleep(4000);

            await device.sdo.download({
                deviceId: SLAVE_ID, index: 0x1F51, subIndex: 1,
                data: 0x80, dataType: canopen.DataType.UNSIGNED8, timeout: 15000 
            });
            console.log("✅ OTA 升级成功锁定");

            console.log("\n--- 🏁 运行功能回归测试 ---");
            device.nmt.startNode(SLAVE_ID); 
            await sleep(2000);

            const val = await device.sdo.upload({ 
                deviceId: SLAVE_ID, index: 0x2102, subIndex: 0, 
                dataType: canopen.DataType.UNSIGNED32, timeout: 15000 
            });
            console.log(`✅ 验证成功！业务数据 [0x2102] = ${val}`);
            console.log("\n🎉 [SUCCESS] 测试成功！");
        } else {
            throw new Error("从站重启后未能检测到心跳");
        }
    } catch (e) {
        console.log("❌ 流程中断: " + (e as any).message);
    }
}

Util.Init(() => {
    device.eds.addSdoClientParameter(SLAVE_ID, 0x600 + SLAVE_ID, 0x580 + SLAVE_ID);
    device.eds.addHeartbeatConsumer(SLAVE_ID, 1000);

    device.nmt.on('heartbeat', (nodeId: number) => {
        if (nodeId === SLAVE_ID && heartbeatResolver) {
            const r = heartbeatResolver; heartbeatResolver = null; r(true);
        }
    });

    device.addListener('message', (message: any) => {
        if (isRebooting) return;
        try {
            const data = (Array.isArray(message.data) || Buffer.isBuffer(message.data)) 
                ? Buffer.from(message.data) : Buffer.from([message.data]);
            (output as any)({ id: message.id, data: data, dir: 'OUT', msgType: { idType: 0 } }).catch(() => {});
        } catch (e) { }
    });

    Util.OnCan(true, (msg: any) => {
        const data = Buffer.from(msg.data);
        if (msg.id === 0x700 + SLAVE_ID && heartbeatResolver) {
            const r = heartbeatResolver; heartbeatResolver = null; r(true);
        }
        if (msg.id === 0x180 + SLAVE_ID) {
            console.log(`📥 [实时 PDO] 0x18A = ${data.readUInt32LE(0)}`);
        }
        try {
            device.receive({ id: msg.id, data: data, len: data.length });
        } catch (e) { }
    });

    device.start();
    device.nmt.startNode(); 
    
    runOtaProcedure();
});