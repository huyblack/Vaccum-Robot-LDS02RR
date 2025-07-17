package com.example.ros2visualizer.view

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.AttributeSet
import android.view.View
import com.example.ros2visualizer.model.MapModel
import com.example.ros2visualizer.model.OdomModel
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

class MapView(context: Context, attrs: AttributeSet?) : View(context, attrs) {

    // Các "bút vẽ" với màu sắc khác nhau
    private val freeSpacePaint = Paint().apply { color = Color.WHITE }
    private val obstaclePaint = Paint().apply { color = Color.BLACK }
    private val unknownSpacePaint = Paint().apply { color = Color.GRAY }
    private val robotPaint = Paint().apply { color = Color.BLUE }
    private val robotDirectionPaint = Paint().apply {
        color = Color.YELLOW
        strokeWidth = 3f
    }

    // Biến để lưu trữ dữ liệu
    private var mapModel: MapModel? = null
    private var odomModel: OdomModel? = null

    // Biến để tính toán tỉ lệ và căn chỉnh bản đồ
    private var scaleFactor = 1.0f
    private var offsetX = 0f
    private var offsetY = 0f

    /**
     * Cập nhật dữ liệu bản đồ và yêu cầu vẽ lại
     */
    fun updateMap(newMap: MapModel) {
        this.mapModel = newMap
        calculateDrawingParameters()
        invalidate() // Yêu cầu hệ thống gọi onDraw()
    }

    /**
     * Cập nhật dữ liệu vị trí robot và yêu cầu vẽ lại
     */
    fun updateOdom(newOdom: OdomModel) {
        this.odomModel = newOdom
        invalidate() // Yêu cầu hệ thống gọi onDraw()
    }

    /**
     * Được gọi khi kích thước của View thay đổi.
     * Đây là nơi tốt nhất để tính toán các thông số co giãn.
     */
    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        calculateDrawingParameters()
    }

    /**
     * Tính toán tỉ lệ để bản đồ vừa với màn hình và được căn giữa.
     */
    private fun calculateDrawingParameters() {
        val currentMap = mapModel ?: return
        if (currentMap.width == 0 || currentMap.height == 0 || width == 0 || height == 0) return

        val scaleX = width.toFloat() / currentMap.width
        val scaleY = height.toFloat() / currentMap.height
        scaleFactor = min(scaleX, scaleY)

        val scaledWidth = currentMap.width * scaleFactor
        val scaledHeight = currentMap.height * scaleFactor
        offsetX = (width - scaledWidth) / 2f
        offsetY = (height - scaledHeight) / 2f
    }

    /**
     * Đây là nơi phép màu xảy ra. Hàm này vẽ mọi thứ lên Canvas.
     */
    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        // 1. Vẽ nền
        canvas.drawColor(Color.DKGRAY)

        val currentMap = mapModel ?: return

        // Lưu trạng thái hiện tại của canvas
        canvas.save()

        // 2. Căn giữa và co giãn bản đồ
        canvas.translate(offsetX, offsetY)
        canvas.scale(scaleFactor, scaleFactor)

        // 3. Vẽ các ô của bản đồ
        for (y in 0 until currentMap.height) {
            for (x in 0 until currentMap.width) {
                val index = y * currentMap.width + x
                if (index >= currentMap.data.size) continue

                val paint = when (currentMap.data[index]) {
                    0 -> freeSpacePaint
                    100 -> obstaclePaint
                    else -> unknownSpacePaint
                }
                canvas.drawRect(x.toFloat(), y.toFloat(), (x + 1).toFloat(), (y + 1).toFloat(), paint)
            }
        }

        // 4. Vẽ robot
        odomModel?.let { odom ->
            // Chuyển đổi tọa độ mét của robot sang tọa độ lưới của bản đồ
            val robotGridX = (odom.x - currentMap.originX) / currentMap.resolution
            // Tọa độ Y trong ROS map và canvas ngược nhau, cần phải đảo ngược
            val robotGridY = currentMap.height - ((odom.y - currentMap.originY) / currentMap.resolution)

            // Vẽ thân robot (hình tròn)
            // Bán kính robot được điều chỉnh để không bị quá to/nhỏ khi zoom
            val robotRadius = 4f / scaleFactor
            canvas.drawCircle(robotGridX.toFloat(), robotGridY.toFloat(), robotRadius, robotPaint)

            // Vẽ hướng của robot (một đường thẳng)
            val lineLength = 6f / scaleFactor
            val endX = robotGridX + lineLength * cos(odom.theta)
            val endY = robotGridY - lineLength * sin(odom.theta) // Y ngược nên dùng dấu trừ
            canvas.drawLine(robotGridX.toFloat(), robotGridY.toFloat(), endX.toFloat(), endY.toFloat(), robotDirectionPaint)
        }

        // Khôi phục lại trạng thái canvas
        canvas.restore()
    }
}