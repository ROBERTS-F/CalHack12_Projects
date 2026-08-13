// server.js
const express = require("express");
const http = require("http");
const WebSocket = require("ws");
const cors = require("cors");

const PORT = 3000;

// ---------- HTTP (Express) ----------
const app = express();
app.use(cors());
app.use(express.json()); // parse JSON bodies

// health check
app.get("/", (_req, res) => res.type("text").send("OK"));

// lens → server data
app.post("/ingest", (req, res) => {
  console.log("POST /ingest", req.body);
  // echo back so you see it in Lens logs
  res.json({ ok: true, echoed: req.body, ts: Date.now() });
});

// ---------- Create HTTP server + WebSocket on /ws ----------
const server = http.createServer(app);
const wss = new WebSocket.Server({ server, path: "/ws" });

wss.on("connection", (ws, req) => {
  console.log("WS connected from", req.socket.remoteAddress);
  ws.send(JSON.stringify({ hello: "from server" }));

  ws.on("message", (m) => {
    const text = m.toString();
    console.log("WS message:", text);
    // simple echo/ack so Spectacles see a reply
    ws.send(JSON.stringify({ ack: true }));
  });

  ws.on("close", () => console.log("WS closed"));
});

server.listen(PORT, "0.0.0.0", () => {
  console.log("HTTP  : http://localhost:%d", PORT);
  console.log("WS    : ws://localhost:%d/ws", PORT);
});
