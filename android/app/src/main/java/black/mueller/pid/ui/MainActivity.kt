package black.mueller.pid.ui

import android.content.Intent
import android.os.Bundle
import android.view.MenuItem
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.GravityCompat
import androidx.fragment.app.commit
import black.mueller.pid.R
import black.mueller.pid.data.ControllerStore
import black.mueller.pid.data.EspType
import com.google.android.material.appbar.MaterialToolbar
import com.google.android.material.navigation.NavigationView
import androidx.drawerlayout.widget.DrawerLayout
class MainActivity : AppCompatActivity(), NavigationView.OnNavigationItemSelectedListener {
    private lateinit var drawerLayout: DrawerLayout
    private lateinit var navView: NavigationView
    private lateinit var toolbar: MaterialToolbar
    private lateinit var store: ControllerStore
    private val linkMap = mutableMapOf<Int, String>()
    private var nextLinkId = 1000
    private val LINKS_GROUP_ID = 100
    private val APP_GROUP_ID = 200

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        store = ControllerStore(this)
        if (!store.isOnboarded()) {
            startActivity(Intent(this, OnboardingActivity::class.java))
            finish()
            return
        }

        setContentView(R.layout.activity_main)
        toolbar = findViewById(R.id.toolbar)
        drawerLayout = findViewById(R.id.drawer_layout)
        navView = findViewById(R.id.navigation_view)

        toolbar.setNavigationIcon(R.drawable.ic_menu_24)
        val accent = androidx.core.content.ContextCompat.getColor(this, R.color.accent)
        toolbar.setTitleTextColor(accent)
        toolbar.navigationIcon?.setTint(accent)
        toolbar.setNavigationContentDescription(R.string.menu_open_drawer)
        toolbar.setNavigationOnClickListener { drawerLayout.openDrawer(GravityCompat.START) }

        navView.setNavigationItemSelectedListener(this)
        updateHeader()
        rebuildMenu()

        if (savedInstanceState == null) {
            supportFragmentManager.commit { replace(R.id.content_frame, WebFragment.newInstance()) }
        }

        // Handle if launched with request to open drawer
        if (intent?.getBooleanExtra("open_drawer", false) == true) {
            drawerLayout.openDrawer(GravityCompat.START)
            intent.removeExtra("open_drawer")
        }
    }

    private fun updateHeader() {
        val header = navView.getHeaderView(0)
        val current = store.getActiveController()
        val subtitle = header.findViewById<android.widget.TextView>(R.id.headerSubtitle)
        subtitle.text = current?.let { "${it.name}: ${it.url}" } ?: getString(R.string.no_controllers)
    }

    override fun onResume() {
        super.onResume()
        updateHeader()
        rebuildMenu()
    }

    override fun onNewIntent(intent: Intent) {
        super.onNewIntent(intent)
        if (intent.getBooleanExtra("open_drawer", false)) {
            drawerLayout.openDrawer(GravityCompat.START)
            intent.removeExtra("open_drawer")
        }
    }

    override fun onNavigationItemSelected(item: MenuItem): Boolean {
        val url = linkMap[item.itemId]
        if (url != null) {
            val frag = supportFragmentManager.findFragmentById(R.id.content_frame) as? WebFragment
            if (frag != null) {
                // load the selected URL
                val wv = frag.view?.findViewById<android.webkit.WebView>(R.id.webView)
                wv?.loadUrl(url)
            } else {
                supportFragmentManager.commit { replace(R.id.content_frame, WebFragment.newInstance()) }
                // After fragment is created, first page will load and links will refresh
            }
        } else {
            when (item.itemId) {
                R.id.nav_settings -> startActivity(Intent(this, SettingsActivity::class.java))
                R.id.nav_app_info -> startActivity(Intent(this, AppInfoActivity::class.java))
            }
        }
        drawerLayout.closeDrawer(GravityCompat.START)
        return true
    }

    private fun rebuildMenu() {
        val current = store.getActiveController() ?: return
        val base = current.url.removeSuffix("/")
        val menu = navView.menu
        menu.clear()
        linkMap.clear()
        nextLinkId = 1000

        val baseEntries = mutableListOf(
            "Dashboard" to "/",
            "PID-Einstellungen" to "/PID",
            // Align with firmware: "/PID-Tuning" route
            "PID-Tuning" to "/PID-Tuning",
            "Profile" to "/Profile"
        )
        if (current.espType == EspType.ESP32) {
            baseEntries.add("Brew-Control" to "/Brew-Control")
        }
        baseEntries.addAll(
            listOf(
                "Chart" to "/Chart",
                "Fast-Heat-Up" to "/Fast-Heat-Up",
                "Eco" to "/ECO",
                "Info" to "/Info",
                "Service" to "/Service",
                "WiFi" to "/Netzwerk",
                "Firmware" to "/Firmware"
            )
        )
        var order = 0
        for ((title, path) in baseEntries) {
            val href = if (path == "/") base else base + path
            val id = nextLinkId++
            linkMap[id] = href
            val item = menu.add(LINKS_GROUP_ID, id, order, title)
            item.isCheckable = false
            order++
        }

        // Separator by starting a new group for app items
        menu.add(APP_GROUP_ID, R.id.nav_settings, 1000, getString(R.string.drawer_settings))
        menu.add(APP_GROUP_ID, R.id.nav_app_info, 1001, getString(R.string.drawer_app_info))
    }
}
