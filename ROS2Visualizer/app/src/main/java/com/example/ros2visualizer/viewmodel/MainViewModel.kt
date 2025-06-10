package com.example.ros2visualizer.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.ros2visualizer.model.ConnectionState
import com.example.ros2visualizer.model.MapModel
import com.example.ros2visualizer.model.OdomModel
import com.example.ros2visualizer.network.RosWebSocketClient
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

class MainViewModel : ViewModel() {

    private var webSocketClient: RosWebSocketClient? = null

    // State for the IP address input
    private val _ipAddress = MutableStateFlow("192.168.2.27")
    val ipAddress: StateFlow<String> = _ipAddress.asStateFlow()

    // State for the connection status
    private val _connectionState = MutableStateFlow(ConnectionState.IDLE)
    val connectionState: StateFlow<ConnectionState> = _connectionState.asStateFlow()

    // States for map and odom data
    private val _mapData = MutableStateFlow<MapModel?>(null)
    val mapData: StateFlow<MapModel?> = _mapData.asStateFlow()

    private val _odomData = MutableStateFlow<OdomModel?>(null)
    val odomData: StateFlow<OdomModel?> = _odomData.asStateFlow()

    fun onIpAddressChange(newIp: String) {
        _ipAddress.value = newIp
    }

    fun connect() {
        if (_connectionState.value == ConnectionState.CONNECTING || _connectionState.value == ConnectionState.CONNECTED) {
            return
        }

        // Reset previous data
        _mapData.value = null
        _odomData.value = null

        val url = "ws://${_ipAddress.value}:8765"
        webSocketClient = RosWebSocketClient(url)

        viewModelScope.launch {
            webSocketClient?.connectionState?.collect { state ->
                _connectionState.value = state
            }
        }

        viewModelScope.launch {
            webSocketClient?.mapData?.collect { map ->
                _mapData.value = map
            }
        }

        viewModelScope.launch {
            webSocketClient?.odomData?.collect { odom ->
                _odomData.value = odom
            }
        }

        webSocketClient?.connect()
    }

    fun disconnect() {
        webSocketClient?.disconnect()
    }

    override fun onCleared() {
        super.onCleared()
        disconnect()
    }
}
