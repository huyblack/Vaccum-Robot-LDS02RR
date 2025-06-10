package com.example.ros2visualizer

import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.viewModels
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.withTransform
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import com.example.ros2visualizer.model.ConnectionState
import com.example.ros2visualizer.model.MapModel
import com.example.ros2visualizer.model.OdomModel
import com.example.ros2visualizer.ui.theme.ROS2VisualizerTheme
import com.example.ros2visualizer.viewmodel.MainViewModel
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

class MainActivity : ComponentActivity() {

    private val viewModel: MainViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContent {
            ROS2VisualizerTheme {
                RosVisualizerScreen(viewModel)
            }
        }
    }
}

@Composable
fun RosVisualizerScreen(viewModel: MainViewModel) {
    val mapModel by viewModel.mapData.collectAsState()
    val odomModel by viewModel.odomData.collectAsState()
    val ipAddress by viewModel.ipAddress.collectAsState()
    val connectionState by viewModel.connectionState.collectAsState()

    Scaffold { paddingValues ->
        Box(
            modifier = Modifier
                .fillMaxSize()
                .padding(paddingValues)
        ) {
            MapCanvas(
                mapModel = mapModel,
                odomModel = odomModel
            )
            ControlPanel(
                modifier = Modifier
                    .align(Alignment.BottomCenter)
                    .fillMaxWidth()
                    .padding(16.dp),
                ipAddress = ipAddress,
                connectionState = connectionState,
                onIpChange = { viewModel.onIpAddressChange(it) },
                onConnect = { viewModel.connect() },
                onDisconnect = { viewModel.disconnect() }
            )
        }
    }
}

@Composable
fun ControlPanel(
    modifier: Modifier = Modifier,
    ipAddress: String,
    connectionState: ConnectionState,
    onIpChange: (String) -> Unit,
    onConnect: () -> Unit,
    onDisconnect: () -> Unit
) {
    val isConnected = connectionState == ConnectionState.CONNECTED
    val isConnecting = connectionState == ConnectionState.CONNECTING

    Card(
        modifier = modifier,
        elevation = CardDefaults.cardElevation(defaultElevation = 8.dp)
    ) {
        Column(
            modifier = Modifier.padding(16.dp),
            verticalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Text("Robot Control", style = MaterialTheme.typography.titleMedium)
            OutlinedTextField(
                value = ipAddress,
                onValueChange = onIpChange,
                label = { Text("Robot IP Address") },
                modifier = Modifier.fillMaxWidth(),
                singleLine = true,
                enabled = !isConnected && !isConnecting
            )
            Row(
                modifier = Modifier.fillMaxWidth(),
                horizontalArrangement = Arrangement.spacedBy(8.dp)
            ) {
                Button(
                    onClick = onConnect,
                    modifier = Modifier.weight(1f),
                    enabled = !isConnected && !isConnecting
                ) {
                    Text(if (isConnecting) "Connecting..." else "Connect")
                }
                Button(
                    onClick = onDisconnect,
                    modifier = Modifier.weight(1f),
                    enabled = isConnected,
                    colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.error)
                ) {
                    Text("Disconnect")
                }
            }
            Text(
                text = "Status: ${connectionState.name}",
                style = MaterialTheme.typography.bodyMedium,
                fontWeight = FontWeight.Bold,
                color = when (connectionState) {
                    ConnectionState.CONNECTED -> Color(0xFF388E3C) // Green
                    ConnectionState.ERROR -> MaterialTheme.colorScheme.error
                    else -> MaterialTheme.colorScheme.onSurface
                }
            )
        }
    }
}


@Composable
fun MapCanvas(mapModel: MapModel?, odomModel: OdomModel?) {
    Canvas(modifier = Modifier.fillMaxSize()) {
        // 1. Vẽ nền
        drawRect(color = Color.DarkGray)

        val currentMap = mapModel ?: return@Canvas
        if (currentMap.width == 0 || currentMap.height == 0 || currentMap.resolution <= 0) return@Canvas

        // 2. Tính toán tỉ lệ và vị trí để căn giữa bản đồ
        val scaleX = size.width / currentMap.width
        val scaleY = size.height / currentMap.height
        val scaleFactor = min(scaleX, scaleY)

        val scaledWidth = currentMap.width * scaleFactor
        val scaledHeight = currentMap.height * scaleFactor
        val offsetX = (size.width - scaledWidth) / 2f
        val offsetY = (size.height - scaledHeight) / 2f

        // 3. Áp dụng các biến đổi (di chuyển và co giãn) lên canvas
        withTransform({
            translate(left = offsetX, top = offsetY)
            scale(scaleX = scaleFactor, scaleY = scaleFactor, pivot = Offset.Zero)
        }) {
            // 4. Vẽ từng ô của bản đồ
            for (y in 0 until currentMap.height) {
                for (x in 0 until currentMap.width) {
                    val index = y * currentMap.width + x
                    if (index >= currentMap.data.size) continue

                    val color = when (currentMap.data[index]) {
                        0 -> Color.White // Không gian trống
                        100 -> Color.Black // Chướng ngại vật
                        else -> Color.Gray // Không gian chưa biết
                    }
                    drawRect(
                        color = color,
                        topLeft = Offset(x.toFloat(), y.toFloat()),
                        size = Size(1f, 1f)
                    )
                }
            }

            // 5. Vẽ robot
            odomModel?.let { odom ->
                // Chuyển đổi tọa độ thế giới (mét) của robot sang tọa độ lưới của bản đồ (pixel)
                val robotGridX = (odom.x - currentMap.originX) / currentMap.resolution
                val robotGridY = currentMap.height - ((odom.y - currentMap.originY) / currentMap.resolution)

                val robotCenter = Offset(robotGridX, robotGridY)

                // TĂNG KÍCH THƯỚC ROBOT TẠI ĐÂY
                val robotRadius = 10f / scaleFactor // Tăng bán kính robot
                val lineLength = 16f / scaleFactor  // Tăng độ dài đường chỉ hướng

                // Vẽ thân robot (vòng tròn màu đỏ)
                drawCircle(
                    color = Color.Red,
                    radius = robotRadius,
                    center = robotCenter
                )

                // Vẽ đường chỉ hướng của robot (màu trắng)
                val screenAngleRad = -odom.theta
                val endX = robotCenter.x + lineLength * cos(screenAngleRad)
                val endY = robotCenter.y + lineLength * sin(screenAngleRad)
                drawLine(
                    color = Color.White,
                    start = robotCenter,
                    end = Offset(endX, endY),
                    strokeWidth = 4f / scaleFactor // Tăng độ dày đường chỉ hướng
                )
            }
        }
    }
}