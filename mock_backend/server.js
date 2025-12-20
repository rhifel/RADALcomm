const express = require("express");
const mongoose = require("mongoose");

const app = express();
app.use(express.json());

mongoose.connect("mongodb").then(()=>console.log("Connected to MongoDb"));

const Event = mongoose.model("Event", new mongoose.Schema({
  type: Number,
  handheld_id: Number,
  tower_id: Number,
  lat: Number,
  lon: Number,
  status: Number,
  status_str: String,
  msg_id: Number,
  createdAt: { type: Date, default: Date.now }
}));

app.post("/api/events", async (req, res) => {
  const event = await Event.create(req.body);
  console.log(event) 
  res.status(201).json({
    success:true,
    event
  });

});

app.get("/api/events", async (req, res) => {
  const events = await Event.find().sort({ createdAt: -1 });
  res.json(events);
});

app.listen(5000, () => {
  console.log("Server running on port 5000");
});