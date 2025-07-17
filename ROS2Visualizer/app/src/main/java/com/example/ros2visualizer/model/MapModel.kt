package com.example.ros2visualizer.model

data class MapModel(
    val width: Int = 0,
    val height: Int = 0,
    val resolution: Double = 0.0,
    val originX: Double = 0.0,
    val originY: Double = 0.0,
    val data: List<Int> = emptyList()
)