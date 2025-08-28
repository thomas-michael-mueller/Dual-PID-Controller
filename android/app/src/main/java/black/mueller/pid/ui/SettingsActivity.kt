package black.mueller.pid.ui

import android.os.Bundle
import android.view.MotionEvent
import android.view.View
import android.view.ViewGroup
import android.text.InputType
import android.view.LayoutInflater
import android.widget.EditText
import android.widget.TextView
import android.widget.Spinner
import android.widget.ArrayAdapter
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import black.mueller.pid.R
import black.mueller.pid.data.Controller
import black.mueller.pid.data.ControllerStore
import black.mueller.pid.data.EspType
import com.google.android.material.appbar.MaterialToolbar

class SettingsActivity : AppCompatActivity() {
    private lateinit var store: ControllerStore
    private lateinit var list: RecyclerView
    private lateinit var adapter: ControllerAdapter
    private val edgeSwipe by lazy { EdgeSwipeDrawerDelegate(this) }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_settings)
        store = ControllerStore(this)

        val toolbar: MaterialToolbar = findViewById(R.id.toolbar)
        toolbar.title = getString(R.string.app_name)
        toolbar.setNavigationIcon(R.drawable.ic_menu_24)
        val accent = androidx.core.content.ContextCompat.getColor(this, R.color.accent)
        toolbar.setTitleTextColor(accent)
        toolbar.navigationIcon?.setTint(accent)
        toolbar.setNavigationOnClickListener {
            val intent = android.content.Intent(this, MainActivity::class.java)
            intent.addFlags(android.content.Intent.FLAG_ACTIVITY_CLEAR_TOP or android.content.Intent.FLAG_ACTIVITY_SINGLE_TOP)
            intent.putExtra("open_drawer", true)
            startActivity(intent)
            finish()
        }

        list = findViewById(R.id.controllerList)
        list.layoutManager = LinearLayoutManager(this)
        adapter = ControllerAdapter(
            onSetActive = { id -> store.setActiveController(id); refresh() },
            onEdit = { c -> showEditDialog(c) },
            onDelete = { id -> store.deleteController(id); refresh() }
        )
        list.adapter = adapter

        findViewById<View>(R.id.addController).setOnClickListener { showAddDialog() }

        refresh()

        // Linke Rand-Wischgeste: Drawer in MainActivity öffnen
        enableEdgeSwipeToOpenDrawer()
    }

    override fun dispatchTouchEvent(ev: MotionEvent): Boolean {
        if (edgeSwipe.onDispatchTouchEvent(ev)) return true
        return super.dispatchTouchEvent(ev)
    }

    private fun refresh() {
        val items = store.listControllers()
        adapter.submit(items, store.getActiveControllerId())
    }

    private fun showAddDialog() {
        val content = layoutInflater.inflate(android.R.layout.simple_list_item_2, null)
        val name = EditText(this).apply { hint = getString(R.string.hint_controller_name) }
        val url = EditText(this).apply { hint = getString(R.string.hint_controller_url); inputType = InputType.TYPE_TEXT_VARIATION_URI }
        val espSpinner = Spinner(this).apply {
            adapter = ArrayAdapter(this@SettingsActivity, android.R.layout.simple_spinner_dropdown_item, listOf(getString(R.string.esp32), getString(R.string.esp8266)))
        }
        val container = androidx.appcompat.widget.LinearLayoutCompat(this).apply {
            orientation = androidx.appcompat.widget.LinearLayoutCompat.VERTICAL
            setPadding(32, 8, 32, 0)
            addView(name)
            addView(url)
            addView(TextView(this@SettingsActivity).apply { text = getString(R.string.label_esp_type) })
            addView(espSpinner)
        }
        AlertDialog.Builder(this)
            .setTitle(getString(R.string.action_add_controller))
            .setView(container)
            .setPositiveButton(R.string.dialog_save) { d, _ ->
                val n = name.text.toString().trim()
                val u = url.text.toString().trim()
                if (n.isNotEmpty() && isValidUrl(u)) {
                    val esp = if (espSpinner.selectedItemPosition == 1) EspType.ESP8266 else EspType.ESP32
                    store.addController(n, u, esp)
                    refresh()
                } else {
                    android.widget.Toast.makeText(this, R.string.invalid_url, android.widget.Toast.LENGTH_SHORT).show()
                }
                d.dismiss()
            }
            .setNegativeButton(R.string.dialog_cancel, null)
            .show()
    }

    private fun showEditDialog(controller: Controller) {
        val name = EditText(this).apply { hint = getString(R.string.hint_controller_name); setText(controller.name) }
        val url = EditText(this).apply { hint = getString(R.string.hint_controller_url); inputType = InputType.TYPE_TEXT_VARIATION_URI; setText(controller.url) }
        val espSpinner = Spinner(this).apply {
            adapter = ArrayAdapter(this@SettingsActivity, android.R.layout.simple_spinner_dropdown_item, listOf(getString(R.string.esp32), getString(R.string.esp8266)))
            setSelection(if (controller.espType == EspType.ESP8266) 1 else 0)
        }
        val isDemo = controller.url.removeSuffix("/").equals(ControllerStore.DEMO_URL.removeSuffix("/"), ignoreCase = true)
        if (isDemo) {
            espSpinner.isEnabled = false
        }
        val container = androidx.appcompat.widget.LinearLayoutCompat(this).apply {
            orientation = androidx.appcompat.widget.LinearLayoutCompat.VERTICAL
            setPadding(32, 8, 32, 0)
            addView(name)
            addView(url)
            addView(TextView(this@SettingsActivity).apply { text = getString(R.string.label_esp_type) })
            addView(espSpinner)
            if (isDemo) {
                addView(TextView(this@SettingsActivity).apply { text = getString(R.string.demo_esp_locked) })
            }
        }
        AlertDialog.Builder(this)
            .setTitle(getString(R.string.edit))
            .setView(container)
            .setPositiveButton(R.string.dialog_save) { d, _ ->
                val n = name.text.toString().trim()
                val u = url.text.toString().trim()
                if (n.isNotEmpty() && isValidUrl(u)) {
                    controller.name = n
                    controller.url = u
                    if (!isDemo) {
                        controller.espType = if (espSpinner.selectedItemPosition == 1) EspType.ESP8266 else EspType.ESP32
                    } else {
                        controller.espType = EspType.ESP32
                    }
                    store.updateController(controller)
                    refresh()
                } else {
                    android.widget.Toast.makeText(this, R.string.invalid_url, android.widget.Toast.LENGTH_SHORT).show()
                }
                d.dismiss()
            }
            .setNegativeButton(R.string.dialog_cancel, null)
            .show()
    }

    private fun isValidUrl(url: String): Boolean {
        val u = url.trim()
        return u.isNotEmpty() && !u.contains(" ")
    }
}

private fun SettingsActivity.enableEdgeSwipeToOpenDrawer() { /* ersetzt durch EdgeSwipeDrawerDelegate */ }

private class ControllerAdapter(
    val onSetActive: (String) -> Unit,
    val onEdit: (Controller) -> Unit,
    val onDelete: (String) -> Unit,
) : RecyclerView.Adapter<ControllerVH>() {
    private val items = mutableListOf<Controller>()
    private var activeId: String? = null

    fun submit(list: List<Controller>, active: String?) {
        items.clear()
        items.addAll(list)
        activeId = active
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ControllerVH {
        val v = LayoutInflater.from(parent.context).inflate(R.layout.item_controller, parent, false)
        return ControllerVH(v)
    }

    override fun getItemCount(): Int = items.size

    override fun onBindViewHolder(holder: ControllerVH, position: Int) {
        val item = items[position]
        holder.name.text = buildString {
            append(item.name)
            if (item.id == activeId) append("  • aktiv")
        }
        holder.url.text = item.url
        holder.btnSetActive.setOnClickListener { onSetActive(item.id) }
        holder.btnEdit.setOnClickListener { onEdit(item) }
        holder.btnDelete.setOnClickListener { onDelete(item.id) }
    }
}

private class ControllerVH(v: View) : RecyclerView.ViewHolder(v) {
    val name: TextView = v.findViewById(R.id.name)
    val url: TextView = v.findViewById(R.id.url)
    val btnSetActive: View = v.findViewById(R.id.btnSetActive)
    val btnEdit: View = v.findViewById(R.id.btnEdit)
    val btnDelete: View = v.findViewById(R.id.btnDelete)
}
