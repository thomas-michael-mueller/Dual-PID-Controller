package black.mueller.pid.ui

import android.os.Bundle
import android.view.MotionEvent
import androidx.appcompat.app.AppCompatActivity
import black.mueller.pid.R
import com.google.android.material.appbar.MaterialToolbar

class AppInfoActivity : AppCompatActivity() {
    private val edgeSwipe by lazy { EdgeSwipeDrawerDelegate(this) }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_app_info)

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

        val pInfo = packageManager.getPackageInfo(packageName, 0)
        val versionText = findViewById<android.widget.TextView>(R.id.versionText)
        versionText.text = pInfo.versionName
    }

    override fun dispatchTouchEvent(ev: MotionEvent): Boolean {
        if (edgeSwipe.onDispatchTouchEvent(ev)) return true
        return super.dispatchTouchEvent(ev)
    }
}

