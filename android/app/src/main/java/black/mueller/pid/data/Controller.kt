package black.mueller.pid.data

data class Controller(
    val id: String,
    var name: String,
    var url: String,
    var espType: EspType = EspType.ESP32,
)
