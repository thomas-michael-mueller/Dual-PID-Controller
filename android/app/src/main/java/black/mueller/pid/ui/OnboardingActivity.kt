package black.mueller.pid.ui

import android.content.Intent
import android.os.Bundle
import android.text.InputType
import android.widget.Button
import android.widget.EditText
import android.widget.Spinner
import android.widget.ArrayAdapter
import androidx.appcompat.app.AppCompatActivity
import black.mueller.pid.R
import black.mueller.pid.data.ControllerStore
import black.mueller.pid.data.EspType

class OnboardingActivity : AppCompatActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_onboarding)

        val inputName: EditText = findViewById(R.id.inputName)
        val inputUrl: EditText = findViewById(R.id.inputUrl)
        inputUrl.inputType = InputType.TYPE_TEXT_VARIATION_URI
        val inputEsp: Spinner = findViewById(R.id.inputEspType)

        // Setup spinner entries
        val espEntries = listOf(getString(R.string.esp32), getString(R.string.esp8266))
        inputEsp.adapter = ArrayAdapter(this, android.R.layout.simple_spinner_dropdown_item, espEntries)

        findViewById<Button>(R.id.btnSave).setOnClickListener {
            val name = inputName.text.toString().trim()
            val url = inputUrl.text.toString().trim()
            if (name.isNotEmpty() && url.isNotEmpty()) {
                val store = ControllerStore(this)
                val esp = if (inputEsp.selectedItemPosition == 1) EspType.ESP8266 else EspType.ESP32
                val c = store.addController(name, url, esp)
                store.setActiveController(c.id)
                startActivity(Intent(this, MainActivity::class.java))
                finish()
            } else {
                android.widget.Toast.makeText(this, R.string.invalid_url, android.widget.Toast.LENGTH_SHORT).show()
            }
        }

        // Demo-Controller verbinden
        findViewById<Button>(R.id.btnDemo)?.setOnClickListener {
            val store = ControllerStore(this)
            val name = "Demo"
            val url = ControllerStore.DEMO_URL
            val c = store.addController(name, url, EspType.ESP32)
            store.setActiveController(c.id)
            startActivity(Intent(this, MainActivity::class.java))
            finish()
        }
    }
}
