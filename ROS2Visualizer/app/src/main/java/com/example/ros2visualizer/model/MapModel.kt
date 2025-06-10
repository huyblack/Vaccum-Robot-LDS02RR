package com.example.ros2visualizer.model

data class MapModel(
    val width: Int,
    val height: Int,
    val resolution: Float,
    val originX: Float,
    val originY: Float,
    val data: List<Int>
)