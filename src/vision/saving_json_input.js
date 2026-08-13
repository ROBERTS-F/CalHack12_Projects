// server.js
const express = require("express");
const http = require("http");
const WebSocket = require("ws");
const cors = require("cors");
const fs = require("fs");
const fsp = fs.promises;
const path = require("path");

const PORT = 3000;

// ---------- Status persistence ----------
const DATA_DIR = path.resolve(__dirname, "data");
const STATUS_PATH = path.join(DATA_DIR, "status.json");

const status = {
  startedAt: new Date().toISOString(),
  http: { count: 0, lastBody: null, lastAt: null, lastIP: null },
  ws: {
    connections: 0,
    messages: 0,
    lastMessageText: null,
    lastAt: null,
    lastIP: null,
  },
};

async function ensureDir(p) {
  await fsp.mkdir(p, { recursive: true });
}

async function writeStatusAtomic(obj) {
  await ensureDir(DATA_DIR);
  const tmp = STATUS_PATH + ".tmp";
  const json = JSON.stringify(obj, null, 2);
  await fsp.writeFile(tmp, json);
  await fsp.rename(tmp, STATUS_PATH);
}

// throttle disk writes a bit (coalesce bursts)
let writeTimer = null;
function scheduleWrite() {
  if (writeTimer) return;
  writeTimer = setTimeout(async () => {
    writeTimer = null;
    try {
      await writeStatusAtomic(status);
    } catch (e) {
      console.error("Failed to write status.json:", e);
    }
  }, 50); // ~20 writes/sec max
}

// ---------- HTTP (Express) ----------
const app = express();
app.use(cors());
app.use(express.json()); // parse JSON bodies

// health check
app.get("/", (_req, res) => res.type("text").send("OK"));

// read latest status
app.get("/status", async (_req, res) => {
  try {
    // Prefer in-memory; if you want to read the file instead, swap these lines:
    // const file = await fsp.readFile(STATUS_PATH, "utf8");
    // return res.type("application/json").send(file);
    res.json(status);
  } catch (e) {
    res.status(500).json({ ok: false, error: String(e) });
  }
});

// lens → server data
app.post("/ingest", (req, res) => {
  const now = new Date();
  status.http.count += 1;
  status.http.lastBody = req.body ?? null;
  status.http.lastAt = now.toISOString();
  status.http.lastIP = req.ip;
  scheduleWrite();

  console.log("POST /ingest", req.body);
  // echo back so you see it in Lens logs
  res.json({ ok: true, echoed: req.body, ts: now.getTime() });
});

// ---------- Create HTTP server + WebSocket on /ws ----------
const server = http.createServer(app);
const wss = new WebSocket.Server({ server, path: "/ws" });

wss.on("connection", (ws, req) => {
  const ip = req.socket.remoteAddress;
  status.ws.connections += 1;
  status.ws.lastIP = ip;
  scheduleWrite();

  console.log("WS connected from", ip);
  ws.send(JSON.stringify({ hello: "from server" }));

  ws.on("message", (m) => {
    const text = m.toString();
    status.ws.messages += 1;
    status.ws.lastMessageText = text.slice(0, 1000); // avoid giant dumps
    status.ws.lastAt = new Date().toISOString();
    scheduleWrite();

    console.log("WS message:", text);
    // simple echo/ack so Spectacles see a reply
    ws.send(JSON.stringify({ ack: true }));
  });

  ws.on("close", () => {
    console.log("WS closed");
  });
});

// write an initial status.json on boot
writeStatusAtomic(status).catch((e) =>
  console.error("Initial status write failed:", e)
);

server.listen(PORT, "0.0.0.0", () => {
  console.log("HTTP  : http://localhost:%d", PORT);
  console.log("WS    : ws://localhost:%d/ws", PORT);
  console.log("STATUS: %s", STATUS_PATH);
});
