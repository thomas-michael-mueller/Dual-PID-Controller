package black.mueller.pid.data

import android.content.Context
import android.content.SharedPreferences
import org.json.JSONArray
import org.json.JSONObject
import java.util.UUID

class ControllerStore(context: Context) {
    private val prefs: SharedPreferences =
        context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE)

    fun listControllers(): List<Controller> {
        val json = prefs.getString(KEY_LIST, "[]") ?: "[]"
        val arr = JSONArray(json)
        val list = mutableListOf<Controller>()
        for (i in 0 until arr.length()) {
            val o = arr.getJSONObject(i)
            list.add(
                Controller(
                    id = o.getString("id"),
                    name = o.getString("name"),
                    url = o.getString("url"),
                    espType = try {
                        EspType.valueOf(o.optString("espType", EspType.ESP32.name))
                    } catch (_: Throwable) {
                        EspType.ESP32
                    }
                )
            )
        }
        return list
    }

    fun saveControllers(list: List<Controller>) {
        val arr = JSONArray()
        list.forEach { c ->
            val o = JSONObject()
            o.put("id", c.id)
            o.put("name", c.name)
            o.put("url", c.url)
            o.put("espType", c.espType.name)
            arr.put(o)
        }
        prefs.edit().putString(KEY_LIST, arr.toString()).apply()
    }

    fun addController(name: String, url: String, espType: EspType = EspType.ESP32): Controller {
        val list = listControllers().toMutableList()
        val controller = Controller(UUID.randomUUID().toString(), name, normalizeUrl(url), espType)
        list.add(controller)
        saveControllers(list)
        if (!prefs.contains(KEY_ACTIVE_ID)) setActiveController(controller.id)
        return controller
    }

    fun updateController(controller: Controller) {
        val list = listControllers().toMutableList()
        val idx = list.indexOfFirst { it.id == controller.id }
        if (idx >= 0) {
            list[idx] = controller.copy(url = normalizeUrl(controller.url), espType = controller.espType)
            saveControllers(list)
        }
    }

    fun deleteController(id: String) {
        val list = listControllers().toMutableList()
        val idx = list.indexOfFirst { it.id == id }
        if (idx >= 0) {
            list.removeAt(idx)
            saveControllers(list)
        }
        if (getActiveControllerId() == id) {
            prefs.edit().remove(KEY_ACTIVE_ID).apply()
            // set first as active if exists
            list.firstOrNull()?.let { setActiveController(it.id) }
        }
    }

    fun setActiveController(id: String) {
        prefs.edit().putString(KEY_ACTIVE_ID, id).apply()
    }

    fun getActiveControllerId(): String? = prefs.getString(KEY_ACTIVE_ID, null)

    fun getActiveController(): Controller? =
        getActiveControllerId()?.let { id -> listControllers().firstOrNull { it.id == id } }

    fun isOnboarded(): Boolean = listControllers().isNotEmpty()

    private fun normalizeUrl(url: String): String {
        var u = url.trim()
        if (!u.matches(Regex("^[a-zA-Z][a-zA-Z0-9+.-]*://.*"))) {
            u = "http://" + u
        }
        return u.removeSuffix("/")
    }

    companion object {
        private const val PREFS_NAME = "controllers_prefs"
        private const val KEY_LIST = "controllers"
        private const val KEY_ACTIVE_ID = "active_controller_id"
        const val DEMO_URL = "https://pid-demo.mueller.black/"
    }
}
