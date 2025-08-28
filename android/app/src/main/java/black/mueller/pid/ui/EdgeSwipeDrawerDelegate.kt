package black.mueller.pid.ui

import android.app.Activity
import android.content.Intent
import android.view.MotionEvent
import android.view.ViewConfiguration

/**
 * Delegiert eine einfache linke Rand-Wischgeste, um die MainActivity mit geöffnetem Drawer zu starten.
 * In der Activity:
 *  - Erzeuge eine Instanz in onCreate: edgeSwipe = EdgeSwipeDrawerDelegate(this)
 *  - Überschreibe dispatchTouchEvent und rufe edgeSwipe.onDispatchTouchEvent(ev).
 */
class EdgeSwipeDrawerDelegate(private val activity: Activity) {
    private val density = activity.resources.displayMetrics.density
    private val edgeWidthPx = (24 * density)
    private val triggerDistancePx = (64 * density)
    private val touchSlop = ViewConfiguration.get(activity).scaledTouchSlop

    private var tracking = false
    private var startX = 0f
    private var startY = 0f

    fun onDispatchTouchEvent(ev: MotionEvent): Boolean {
        when (ev.actionMasked) {
            MotionEvent.ACTION_DOWN -> {
                tracking = ev.x <= edgeWidthPx
                startX = ev.x
                startY = ev.y
            }
            MotionEvent.ACTION_MOVE -> {
                if (tracking) {
                    val dx = ev.x - startX
                    val dy = kotlin.math.abs(ev.y - startY)
                    // Horizontaler Swipe mit leichter Vertikal-Toleranz
                    if (dx > triggerDistancePx && dx > dy + touchSlop) {
                        openDrawer()
                        tracking = false
                        return true
                    }
                }
            }
            MotionEvent.ACTION_UP, MotionEvent.ACTION_CANCEL -> {
                tracking = false
            }
        }
        return false
    }

    private fun openDrawer() {
        val intent = Intent(activity, MainActivity::class.java)
        intent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP or Intent.FLAG_ACTIVITY_SINGLE_TOP)
        intent.putExtra("open_drawer", true)
        activity.startActivity(intent)
        activity.finish()
    }
}

