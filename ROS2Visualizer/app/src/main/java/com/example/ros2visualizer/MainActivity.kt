package com.example.ros2visualizer

import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.viewModels
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.detectTransformGestures
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.withTransform
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.layout.onGloballyPositioned
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.IntSize
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
    // Các biến state để lưu trữ trạng thái pan và zoom
    var scale by remember { mutableFloatStateOf(1f) }
    var offset by remember { mutableStateOf(Offset.Zero) }
    var canvasSize by remember { mutableStateOf(IntSize.Zero) }

    // Biến cờ để chỉ thực hiện tính toán ban đầu một lần
    var isInitialCalculationDone by remember { mutableStateOf(false) }

    // Tự động tính toán để căn giữa và phóng to bản đồ khi nó xuất hiện lần đầu.
    // Effect này sẽ chạy khi mapModel hoặc kích thước canvas thay đổi.
    LaunchedEffect(mapModel, canvasSize) {
        if (mapModel != null && !isInitialCalculationDone && canvasSize != IntSize.Zero) {
            val mapWidth = mapModel.width.toFloat()
            val mapHeight = mapModel.height.toFloat()

            if (mapWidth <= 0f || mapHeight <= 0f) return@LaunchedEffect

            val scaleX = canvasSize.width / mapWidth
            val scaleY = canvasSize.height / mapHeight
            val calculatedScale = min(scaleX, scaleY) * 0.9f // Để lại chút lề

            val scaledWidth = mapWidth * calculatedScale
            val scaledHeight = mapHeight * calculatedScale
            val calculatedOffsetX = (canvasSize.width - scaledWidth) / 2f
            val calculatedOffsetY = (canvasSize.height - scaledHeight) / 2f

            scale = calculatedScale
            offset = Offset(calculatedOffsetX, calculatedOffsetY)
            isInitialCalculationDone = true
        }
    }

    Canvas(
        modifier = Modifier
            .fillMaxSize()
            .onGloballyPositioned { coordinates ->
                // Lấy kích thước của Canvas khi nó được vẽ
                canvasSize = coordinates.size
            }
            .pointerInput(Unit) {
                detectTransformGestures { _, pan, zoom, _ ->
                    // Khi người dùng tương tác, đánh dấu là đã tương tác để không reset view
                    isInitialCalculationDone = true
                    // Cập nhật scale và offset dựa trên cử chỉ của người dùng
                    scale = (scale * zoom).coerceIn(0.1f, 50f) // Tăng giới hạn zoom tối đa
                    offset += pan
                }
            }
    ) {
        // 1. Vẽ nền
        drawRect(color = Color.DarkGray)

        val currentMap = mapModel ?: return@Canvas
        if (currentMap.width == 0 || currentMap.height == 0 || currentMap.resolution <= 0) return@Canvas

        // 2. Áp dụng các biến đổi (di chuyển và co giãn) lên canvas
        withTransform({
            translate(left = offset.x, top = offset.y)
            scale(scale, scale, pivot = Offset.Zero)
        }) {
            // 3. Vẽ từng ô của bản đồ
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

            // 4. Vẽ robot
            odomModel?.let { odom ->
                // Chuyển đổi tọa độ thế giới (mét) của robot sang tọa độ lưới của bản đồ (pixel)
                val robotGridX = ((odom.x - currentMap.originX) / currentMap.resolution).toFloat()
                val robotGridY =
                    (currentMap.height - ((odom.y - currentMap.originY) / currentMap.resolution)).toFloat()

                val robotCenter = Offset(robotGridX, robotGridY)

                // Kích thước của robot sẽ được điều chỉnh ngược lại với mức zoom
                // để robot trông có vẻ có kích thước không đổi trên màn hình.
                val robotRadius = 10f / scale
                val lineLength = 16f / scale
                val strokeWidth = 4f / scale

                // Vẽ thân robot (vòng tròn màu đỏ)
                drawCircle(
                    color = Color.Red,
                    radius = robotRadius,
                    center = robotCenter
                )

                // Vẽ đường chỉ hướng của robot (màu trắng)
                val screenAngleRad = -odom.theta
                val endX = robotCenter.x + lineLength * cos(screenAngleRad).toFloat()
                val endY = robotCenter.y + lineLength * sin(screenAngleRad).toFloat()
                drawLine(
                    color = Color.White,
                    start = robotCenter,
                    end = Offset(endX, endY),
                    strokeWidth = strokeWidth
                )
            }
        }
    }
}