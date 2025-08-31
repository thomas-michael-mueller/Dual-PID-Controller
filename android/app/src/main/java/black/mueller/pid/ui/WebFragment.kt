package black.mueller.pid.ui

import android.annotation.SuppressLint
import android.graphics.Bitmap
import android.Manifest
import android.app.Activity
import android.content.Intent
import android.net.Uri
import android.os.Build
import android.webkit.CookieManager
import android.webkit.DownloadListener
import android.webkit.URLUtil
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.core.content.ContextCompat
import android.content.pm.PackageManager
import android.app.DownloadManager
import android.os.Environment
import android.net.http.SslError
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.webkit.SslErrorHandler
import android.webkit.WebChromeClient
import android.webkit.WebResourceError
import android.webkit.WebResourceRequest
import android.webkit.WebSettings
import android.webkit.WebView
import android.webkit.WebViewClient
import android.net.ConnectivityManager
import android.content.Context
import com.airbnb.lottie.LottieAnimationView
import com.google.android.material.progressindicator.LinearProgressIndicator
import androidx.fragment.app.Fragment
import black.mueller.pid.R
import black.mueller.pid.data.ControllerStore
import black.mueller.pid.utils.NetworkUtils

class WebFragment : Fragment() {
    private lateinit var webView: WebView
    private lateinit var progress: View
    private lateinit var errorView: View
    private lateinit var errorTitle: android.widget.TextView
    private lateinit var errorMessage: android.widget.TextView
    private lateinit var openWifiSettingsButton: View
    private var errorIcon: android.widget.ImageView? = null
    private lateinit var errorIconGlyph: android.widget.TextView
    private lateinit var errorLottie: LottieAnimationView
    private lateinit var topProgress: LinearProgressIndicator
    private var restoredFromState: Boolean = false
    private var lastUrl: String? = null
    private var autoRetriedAfterNetwork = false
    private var networkCallback: ConnectivityManager.NetworkCallback? = null
    private var filePathCallback: android.webkit.ValueCallback<Array<Uri>>? = null
    private var pendingDownload: DownloadRequestData? = null

    private val fileChooserLauncher = registerForActivityResult(ActivityResultContracts.StartActivityForResult()) { result ->
        val cb = filePathCallback
        filePathCallback = null
        if (cb == null) return@registerForActivityResult
        if (result.resultCode != Activity.RESULT_OK) {
            cb.onReceiveValue(null)
            return@registerForActivityResult
        }
        val data = result.data
        val uris = mutableListOf<Uri>()
        if (data?.clipData != null) {
            val cd = data.clipData!!
            for (i in 0 until cd.itemCount) {
                val uri = cd.getItemAt(i).uri
                takePersistable(uri)
                uris.add(uri)
            }
        } else if (data?.data != null) {
            takePersistable(data.data!!)
            uris.add(data.data!!)
        }
        cb.onReceiveValue(if (uris.isEmpty()) null else uris.toTypedArray())
    }

    private val requestStoragePermission = registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
        val req = pendingDownload
        pendingDownload = null
        if (granted && req != null) startDownload(req)
        else if (!granted) Toast.makeText(requireContext(), "Download abgebrochen: keine Berechtigung", Toast.LENGTH_SHORT).show()
    }
    

    companion object {
        fun newInstance() = WebFragment()
        private const val STATE_WEBVIEW = "web_state"
        private const val STATE_LAST_URL = "last_url"
        private const val INJECT_CSS = """
            (function(){
              try {
                var css = '*[role=\\"navigation\\"], nav, .nav, .navbar, .navigation, #nav, #navbar, #navigation { display:none !important; }';
                var style = document.createElement('style');
                style.type = 'text/css';
                style.appendChild(document.createTextNode(css));
                document.head.appendChild(style);
              } catch(e) {}
            })();
        """
    }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        val v = inflater.inflate(R.layout.fragment_web, container, false)
        webView = v.findViewById(R.id.webView)
        progress = v.findViewById(R.id.progressBar)
        errorView = v.findViewById(R.id.errorView)
        errorTitle = v.findViewById(R.id.errorTitle)
        errorMessage = v.findViewById(R.id.errorMessage)
        openWifiSettingsButton = v.findViewById(R.id.openWifiSettingsButton)
        errorIconGlyph = v.findViewById(R.id.errorIconGlyph)
        errorLottie = v.findViewById(R.id.errorLottie)
        topProgress = v.findViewById(R.id.topProgress)
        v.findViewById<View>(R.id.retryButton).setOnClickListener { load() }
        v.findViewById<View>(R.id.changeControllerButton).setOnClickListener {
            startActivity(android.content.Intent(requireContext(), SettingsActivity::class.java))
        }
        openWifiSettingsButton.setOnClickListener {
            try {
                startActivity(android.content.Intent(android.provider.Settings.ACTION_WIFI_SETTINGS))
            } catch (_: Throwable) { }
        }
        setupWebView()
        // Evtl. zuletzt geladene URL wiederherstellen (robust gegen SPA, die die URL nicht ändert)
        lastUrl = savedInstanceState?.getString(STATE_LAST_URL)

        // Zuerst versuchen, den WebView-internen Verlauf zu restaurieren
        val webState = savedInstanceState?.getBundle(STATE_WEBVIEW)
        if (webState != null) {
            try {
                webView.restoreState(webState)
                restoredFromState = true
            } catch (_: Throwable) {
                restoredFromState = false
            }
        }

        // Falls der Verlauf nicht wiederhergestellt werden konnte, aber eine letzte URL bekannt ist, diese laden
        if (!restoredFromState && lastUrl != null) {
            webView.loadUrl(lastUrl!!)
            restoredFromState = true
        }

        // Wenn weiterhin nichts geladen ist, Standard-URL laden
        if (!restoredFromState && webView.url == null) {
            load()
        }
        return v
    }

    override fun onResume() {
        super.onResume()
        // Kein automatisches Neuladen, aktuelle Seite beibehalten
    }

    override fun onStart() {
        super.onStart()
        registerNetworkCallback()
    }

    override fun onStop() {
        super.onStop()
        unregisterNetworkCallback()
    }

    @SuppressLint("SetJavaScriptEnabled")
    private fun setupWebView() {
        val s: WebSettings = webView.settings
        s.javaScriptEnabled = true
        s.domStorageEnabled = true
        s.allowFileAccess = true
        s.allowContentAccess = true
        s.loadWithOverviewMode = true
        s.useWideViewPort = true
        s.builtInZoomControls = true
        s.displayZoomControls = false
        s.mixedContentMode = WebSettings.MIXED_CONTENT_ALWAYS_ALLOW

        webView.webChromeClient = object : WebChromeClient() {
            override fun onShowFileChooser(
                view: WebView?,
                filePathCallback: android.webkit.ValueCallback<Array<Uri>>?,
                fileChooserParams: FileChooserParams?
            ): Boolean {
                this@WebFragment.filePathCallback?.onReceiveValue(null)
                this@WebFragment.filePathCallback = filePathCallback

                val accept = (fileChooserParams?.acceptTypes ?: emptyArray()).filter { it.isNotBlank() }
                val allowMultiple = fileChooserParams?.mode == FileChooserParams.MODE_OPEN_MULTIPLE

                // Prefer the system-provided intent; fallback to ACTION_OPEN_DOCUMENT
                val base = try { fileChooserParams?.createIntent() } catch (_: Throwable) { null }
                val intent = if (base != null) {
                    base.addCategory(Intent.CATEGORY_OPENABLE)
                    base.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION or Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION)
                    if (allowMultiple) base.putExtra(Intent.EXTRA_ALLOW_MULTIPLE, true)
                    // Ensure a sane default type
                    if (base.type.isNullOrBlank()) {
                        base.type = if (accept.size == 1) accept[0] else "*/*"
                        if (accept.size > 1) base.putExtra(Intent.EXTRA_MIME_TYPES, accept.toTypedArray())
                    }
                    base
                } else {
                    Intent(Intent.ACTION_OPEN_DOCUMENT).apply {
                        addCategory(Intent.CATEGORY_OPENABLE)
                        addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION or Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION)
                        putExtra(Intent.EXTRA_ALLOW_MULTIPLE, allowMultiple)
                        when {
                            accept.isEmpty() -> type = "*/*"
                            accept.size == 1 -> type = accept[0]
                            else -> {
                                type = "*/*"
                                putExtra(Intent.EXTRA_MIME_TYPES, accept.toTypedArray())
                            }
                        }
                    }
                }

                return try {
                    fileChooserLauncher.launch(intent)
                    true
                } catch (t: Throwable) {
                    this@WebFragment.filePathCallback?.onReceiveValue(null)
                    this@WebFragment.filePathCallback = null
                    false
                }
            }

            override fun onProgressChanged(view: WebView?, newProgress: Int) {
                super.onProgressChanged(view, newProgress)
                try {
                    if (topProgress.isIndeterminate) topProgress.isIndeterminate = false
                    // Farben einmalig setzen (falls Theme keine setzt)
                    if (topProgress.visibility == View.GONE && isAdded) {
                        try {
                            topProgress.setIndicatorColor(ContextCompat.getColor(requireContext(), R.color.accent))
                        } catch (_: Throwable) { }
                    }
                    if (newProgress in 0..99) {
                        if (topProgress.visibility != View.VISIBLE) topProgress.visibility = View.VISIBLE
                        topProgress.setProgressCompat(newProgress, true)
                    } else {
                        topProgress.visibility = View.GONE
                    }
                } catch (_: Throwable) { }
            }
        }
        webView.webViewClient = object : WebViewClient() {
            // Handle legacy Android (< API 23) errors, so we always show our custom error view
            @Suppress("DEPRECATION", "OVERRIDE_DEPRECATION")
            override fun onReceivedError(view: WebView?, errorCode: Int, description: String?, failingUrl: String?) {
                if (!isAdded) return
                val kind = when {
                    errorCode == WebViewClient.ERROR_CONNECT || errorCode == WebViewClient.ERROR_HOST_LOOKUP -> ErrorKind.NETWORK_OFFLINE
                    errorCode == WebViewClient.ERROR_TIMEOUT -> ErrorKind.TIMEOUT
                    !NetworkUtils.isNetworkAvailable(requireContext()) -> ErrorKind.NETWORK_OFFLINE
                    else -> ErrorKind.OTHER
                }
                showError(kind)
            }
            override fun onPageStarted(view: WebView?, url: String?, favicon: Bitmap?) {
                showLoading()
                if (!url.isNullOrEmpty()) {
                    lastUrl = url
                }
            }

            override fun onPageFinished(view: WebView?, url: String?) {
                hideLoading()
                injectCss()
                // Zuletzt geladene URL mitführen (hilft bei Orientierung/Rotation)
                if (!url.isNullOrEmpty()) {
                    lastUrl = url
                }
            }

            override fun onReceivedError(
                view: WebView?, request: WebResourceRequest?, error: WebResourceError?
            ) {
                if (request == null || request.isForMainFrame) {
                    if (!isAdded) return
                    val code = try { error?.errorCode } catch (_: Throwable) { null }
                    val desc = try { error?.description?.toString()?.lowercase() } catch (_: Throwable) { null }
                    val isTimeout = (code == WebViewClient.ERROR_TIMEOUT) || (desc?.contains("timed out") == true) || (desc?.contains("err_timed_out") == true) || (desc?.contains("connection timed out") == true)
                    val kind = when {
                        code == WebViewClient.ERROR_CONNECT || code == WebViewClient.ERROR_HOST_LOOKUP -> ErrorKind.NETWORK_OFFLINE
                        isTimeout -> ErrorKind.TIMEOUT
                        !NetworkUtils.isNetworkAvailable(requireContext()) -> ErrorKind.NETWORK_OFFLINE
                        else -> ErrorKind.OTHER
                    }
                    showError(kind)
                }
            }

            override fun onReceivedHttpError(
                view: WebView?, request: WebResourceRequest?, errorResponse: android.webkit.WebResourceResponse?
            ) {
                if (request == null || request.isForMainFrame) {
                    if (!isAdded) return
                    showError(ErrorKind.HTTP_ERROR)
                }
            }

            override fun onReceivedSslError(view: WebView?, handler: SslErrorHandler?, error: SslError?) {
                // For local controllers with self-signed certs, cancel and show error instead of proceeding silently.
                handler?.cancel()
                if (!isAdded) return
                showError(ErrorKind.SSL_ERROR)
            }
        }
        webView.setDownloadListener(DownloadListener { url, userAgent, contentDisposition, mimeType, contentLength ->
            handleDownload(url ?: return@DownloadListener, userAgent, contentDisposition, mimeType)
        })
    }

    private fun injectCss() {
        try {
            webView.evaluateJavascript(INJECT_CSS, null)
        } catch (_: Throwable) { }
    }

    override fun onSaveInstanceState(outState: Bundle) {
        super.onSaveInstanceState(outState)
        val state = Bundle()
        try {
            webView.saveState(state)
            outState.putBundle(STATE_WEBVIEW, state)
            // Zusätzlich die letzte URL sichern (robust gegen SPAs ohne URL-Wechsel)
            lastUrl = webView.url ?: lastUrl
            if (!lastUrl.isNullOrEmpty()) {
                outState.putString(STATE_LAST_URL, lastUrl)
            }
        } catch (_: Throwable) { }
    }

    private fun load() {
        val store = ControllerStore(requireContext())
        val active = store.getActiveController()
        if (active == null) {
            showError(ErrorKind.OTHER)
            return
        }
        // If there is no network, avoid showing the WebView default error page
        if (!NetworkUtils.isNetworkAvailable(requireContext())) {
            showError(ErrorKind.NETWORK_OFFLINE)
            return
        }
        showLoading()
        webView.loadUrl(active.url)
    }

    fun navigateTo(url: String) {
        showLoading()
        webView.loadUrl(url)
    }

    private fun showLoading() {
        progress.visibility = View.VISIBLE
        errorView.visibility = View.GONE
        webView.visibility = View.INVISIBLE
        errorLottie.playAnimation()
    }

    private fun hideLoading() {
        progress.visibility = View.GONE
        errorView.visibility = View.GONE
        webView.visibility = View.VISIBLE
        if (errorLottie.isAnimating) errorLottie.cancelAnimation()
    }

    private fun showError(kind: ErrorKind) {
        progress.visibility = View.GONE
        errorView.visibility = View.VISIBLE
        webView.visibility = View.GONE
        when (kind) {
            ErrorKind.NETWORK_OFFLINE -> {
                errorTitle.text = getString(R.string.error_network_title)
                errorMessage.text = getString(R.string.error_network_message)
                errorIconGlyph.text = "wifi_off"
                openWifiSettingsButton.visibility = View.VISIBLE
                errorIcon?.visibility = View.GONE
                errorLottie.visibility = View.VISIBLE
            }
            ErrorKind.SSL_ERROR -> {
                errorTitle.text = getString(R.string.error_ssl_title)
                errorMessage.text = getString(R.string.error_ssl_message)
                errorIconGlyph.text = "https"
                openWifiSettingsButton.visibility = View.GONE
                errorIcon?.visibility = View.GONE
                errorLottie.visibility = View.VISIBLE
            }
            ErrorKind.HTTP_ERROR -> {
                errorTitle.text = getString(R.string.error_http_title)
                errorMessage.text = getString(R.string.error_http_message)
                errorIconGlyph.text = "error_outline"
                openWifiSettingsButton.visibility = View.GONE
                errorIcon?.visibility = View.GONE
                errorLottie.visibility = View.VISIBLE
            }
            ErrorKind.TIMEOUT -> {
                errorTitle.text = getString(R.string.error_timeout_title)
                errorMessage.text = getString(R.string.error_timeout_message)
                errorIconGlyph.text = "schedule"
                openWifiSettingsButton.visibility = View.GONE
                errorIcon?.visibility = View.GONE
                errorLottie.visibility = View.VISIBLE
            }
            ErrorKind.OTHER -> {
                errorTitle.text = getString(R.string.error_generic_title)
                errorMessage.text = getString(R.string.error_generic_message)
                errorIconGlyph.text = "error_outline"
                openWifiSettingsButton.visibility = View.GONE
                errorIcon?.visibility = View.GONE
                errorLottie.visibility = View.VISIBLE
            }
        }
    }

    private enum class ErrorKind { NETWORK_OFFLINE, SSL_ERROR, HTTP_ERROR, TIMEOUT, OTHER }
    private data class DownloadRequestData(
        val url: String,
        val userAgent: String?,
        val contentDisposition: String?,
        val mimeType: String?
    )

    private fun handleDownload(url: String, userAgent: String?, contentDisposition: String?, mimeType: String?) {
        if (!isAdded) return
        if (url.startsWith("blob:")) {
            Toast.makeText(requireContext(), "Download aus 'blob:'-URLs wird aktuell nicht unterstützt.", Toast.LENGTH_LONG).show()
            return
        }
        val req = DownloadRequestData(url, userAgent, contentDisposition, mimeType)
        if (Build.VERSION.SDK_INT < 29) {
            val granted = ContextCompat.checkSelfPermission(requireContext(), Manifest.permission.WRITE_EXTERNAL_STORAGE) == PackageManager.PERMISSION_GRANTED
            if (!granted) {
                pendingDownload = req
                requestStoragePermission.launch(Manifest.permission.WRITE_EXTERNAL_STORAGE)
                return
            }
        }
        startDownload(req)
    }

    private fun startDownload(req: DownloadRequestData) {
        val ctx = requireContext()
        val uri = Uri.parse(req.url)
        val filename = URLUtil.guessFileName(req.url, req.contentDisposition, req.mimeType)
        val request = DownloadManager.Request(uri)
        request.setTitle(filename)
        request.setDescription(uri.host ?: "Download")
        req.mimeType?.let { request.setMimeType(it) }
        request.setNotificationVisibility(DownloadManager.Request.VISIBILITY_VISIBLE_NOTIFY_COMPLETED)
        CookieManager.getInstance()?.getCookie(req.url)?.let { request.addRequestHeader("Cookie", it) }
        req.userAgent?.let { request.addRequestHeader("User-Agent", it) }
        try {
            if (Build.VERSION.SDK_INT >= 29) {
                // Public Downloads, keine Laufzeitberechtigung nötig
                request.setDestinationInExternalPublicDir(Environment.DIRECTORY_DOWNLOADS, filename)
            } else {
                request.setDestinationInExternalPublicDir(Environment.DIRECTORY_DOWNLOADS, filename)
            }
        } catch (_: Throwable) {
            // Fallback: App-spezifischer Ordner
            request.setDestinationInExternalFilesDir(ctx, Environment.DIRECTORY_DOWNLOADS, filename)
        }
        val dm = ctx.getSystemService(Context.DOWNLOAD_SERVICE) as DownloadManager
        dm.enqueue(request)
        Toast.makeText(ctx, "Download gestartet: $filename", Toast.LENGTH_SHORT).show()
    }

    private fun takePersistable(uri: Uri) {
        try {
            val flags = Intent.FLAG_GRANT_READ_URI_PERMISSION or Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION
            requireContext().contentResolver.takePersistableUriPermission(uri, flags)
        } catch (_: Throwable) { }
    }

    private fun registerNetworkCallback() {
        val cm = requireContext().getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
        autoRetriedAfterNetwork = false
        val cb = object : ConnectivityManager.NetworkCallback() {
            override fun onAvailable(network: android.net.Network) {
                if (!isAdded) return
                // Nur automatisch neu laden, wenn wir gerade im Error-Screen sind
                val shouldRetry = errorView.visibility == View.VISIBLE && !autoRetriedAfterNetwork
                if (shouldRetry) {
                    autoRetriedAfterNetwork = true
                    requireActivity().runOnUiThread {
                        load()
                    }
                }
            }
        }
        networkCallback = cb
        try {
            cm.registerDefaultNetworkCallback(cb)
        } catch (_: Throwable) { }
    }

    private fun unregisterNetworkCallback() {
        val cm = requireContext().getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
        networkCallback?.let {
            try { cm.unregisterNetworkCallback(it) } catch (_: Throwable) { }
        }
        networkCallback = null
    }
}
