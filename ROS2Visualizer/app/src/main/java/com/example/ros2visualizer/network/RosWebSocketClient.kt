package com.example.ros2visualizer.network
import android.util.Log
import com.example.ros2visualizer.model.ConnectionState
import com.example.ros2visualizer.model.MapModel
import com.example.ros2visualizer.model.OdomModel
import com.google.gson.JsonObject
import com.google.gson.JsonParser
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import okhttp3.*

class RosWebSocketClient(private val url: String) {

    private val client = OkHttpClient()
    private var webSocket: WebSocket? = null

    private val _connectionState = MutableStateFlow(ConnectionState.IDLE)
    val connectionState: StateFlow<ConnectionState> = _connectionState

    private val _mapData = MutableStateFlow<MapModel?>(null)
    val mapData: StateFlow<MapModel?> = _mapData

    private val _odomData = MutableStateFlow<OdomModel?>(null)
    val odomData: StateFlow<OdomModel?> = _odomData

    fun connect() {
        if (_connectionState.value == ConnectionState.CONNECTING || _connectionState.value == ConnectionState.CONNECTED) {
            Log.w("RosWebSocketClient", "Already connecting or connected.")
            return
        }
        _connectionState.value = ConnectionState.CONNECTING
        val request = Request.Builder().url(url).build()
        webSocket = client.newWebSocket(request, RosWebSocketListener())
    }

    fun disconnect() {
        webSocket?.close(1000, "Client disconnected")
    }

    private fun parseMap(mapContainer: JsonObject) {
        try {
            if (mapContainer.has("info") && mapContainer.get("info").isJsonObject &&
                mapContainer.has("data") && mapContainer.get("data").isJsonArray) {

                val info = mapContainer.getAsJsonObject("info")
                val width = info.get("width").asInt
                val height = info.get("height").asInt
                val resolution = info.get("resolution").asDouble

                val origin = info.getAsJsonObject("origin")
                val originX = origin.get("x").asDouble
                val originY = origin.get("y").asDouble

                val data = mapContainer.getAsJsonArray("data").map { it.asInt }

                _mapData.value = MapModel(width, height, resolution, originX, originY, data)
                Log.d("RosData", "Parsed Map with origin: x=$originX, y=$originY")
            }
        } catch (e: Exception) {
            Log.e("RosWebSocketClient", "Error parsing map data", e)
        }
    }

    private fun parseOdom(odomObject: JsonObject) {
        try {
            if (odomObject.has("pose") && odomObject.get("pose").isJsonObject) {
                val pose = odomObject.getAsJsonObject("pose")
                if (pose.has("position") && pose.get("position").isJsonObject && pose.has("orientation_yaw")) {
                    val position = pose.getAsJsonObject("position")
                    val x = position.get("x").asDouble
                    val y = position.get("y").asDouble
                    val theta = pose.get("orientation_yaw").asDouble

                    _odomData.value = OdomModel(x, y, theta)
                    Log.d("RosData", "Parsed Odom: x=$x, y=$y, theta=$theta")
                }
            }
        } catch (e: Exception) {
            Log.e("RosWebSocketClient", "Error parsing odom data", e)
        }
    }


    private inner class RosWebSocketListener : WebSocketListener() {
        override fun onOpen(webSocket: WebSocket, response: Response) {
            Log.d("RosWebSocketClient", "WebSocket Connected")
            _connectionState.value = ConnectionState.CONNECTED
        }

        override fun onMessage(webSocket: WebSocket, text: String) {
            // Log the raw message to debug
            Log.d("RawJson", "Received: $text")
            try {
                val jsonElement = JsonParser.parseString(text)
                if (jsonElement !is JsonObject) {
                    return // Bỏ qua nếu không phải là một đối tượng JSON
                }

                val jsonObject = jsonElement.asJsonObject

                // Phân loại gói tin dựa trên cấu trúc
                if (jsonObject.has("map")) {
                    parseMap(jsonObject.getAsJsonObject("map"))
                } else if (jsonObject.has("type") && jsonObject.get("type").asString == "odom") {
                    parseOdom(jsonObject)
                }

            } catch (e: Exception) {
                Log.e("RosWebSocketClient", "Error parsing incoming JSON string", e)
            }
        }

        override fun onClosing(webSocket: WebSocket, code: Int, reason: String) {
            webSocket.close(1000, null)
            Log.d("RosWebSocketClient", "WebSocket Closing: $code / $reason")
            _connectionState.value = ConnectionState.DISCONNECTED
        }

        override fun onFailure(webSocket: WebSocket, t: Throwable, response: Response?) {
            Log.e("RosWebSocketClient", "WebSocket Error", t)
            _connectionState.value = ConnectionState.ERROR
        }
    }
}