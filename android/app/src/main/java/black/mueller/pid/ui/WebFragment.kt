package black.mueller.pid.ui

import android.annotation.SuppressLint
import android.graphics.Bitmap
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
import androidx.fragment.app.Fragment
import black.mueller.pid.R
import black.mueller.pid.data.ControllerStore
import black.mueller.pid.utils.NetworkUtils

class WebFragment : Fragment() {
    private lateinit var webView: WebView
    private lateinit var progress: View
    private lateinit var errorView: View
    

    companion object {
        fun newInstance() = WebFragment()
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
        v.findViewById<View>(R.id.retryButton).setOnClickListener { load() }
        v.findViewById<View>(R.id.changeControllerButton).setOnClickListener {
            startActivity(android.content.Intent(requireContext(), SettingsActivity::class.java))
        }
        setupWebView()
        return v
    }

    override fun onResume() {
        super.onResume()
        load()
    }

    @SuppressLint("SetJavaScriptEnabled")
    private fun setupWebView() {
        val s: WebSettings = webView.settings
        s.javaScriptEnabled = true
        s.domStorageEnabled = true
        s.loadWithOverviewMode = true
        s.useWideViewPort = true
        s.builtInZoomControls = true
        s.displayZoomControls = false
        s.mixedContentMode = WebSettings.MIXED_CONTENT_ALWAYS_ALLOW

        webView.webChromeClient = object : WebChromeClient() {}
        webView.webViewClient = object : WebViewClient() {
            // Handle legacy Android (< API 23) errors, so we always show our custom error view
            override fun onReceivedError(view: WebView?, errorCode: Int, description: String?, failingUrl: String?) {
                showError()
            }
            override fun onPageStarted(view: WebView?, url: String?, favicon: Bitmap?) {
                showLoading()
            }

            override fun onPageFinished(view: WebView?, url: String?) {
                hideLoading()
                injectCss()
            }

            override fun onReceivedError(
                view: WebView?, request: WebResourceRequest?, error: WebResourceError?
            ) {
                if (request == null || request.isForMainFrame) {
                    showError()
                }
            }

            override fun onReceivedHttpError(
                view: WebView?, request: WebResourceRequest?, errorResponse: android.webkit.WebResourceResponse?
            ) {
                if (request == null || request.isForMainFrame) {
                    showError()
                }
            }

            override fun onReceivedSslError(view: WebView?, handler: SslErrorHandler?, error: SslError?) {
                // For local controllers with self-signed certs, cancel and show error instead of proceeding silently.
                handler?.cancel()
                showError()
            }
        }
    }

    private fun injectCss() {
        try {
            webView.evaluateJavascript(INJECT_CSS, null)
        } catch (_: Throwable) { }
    }


    private fun load() {
        val store = ControllerStore(requireContext())
        val active = store.getActiveController()
        if (active == null) {
            showError()
            return
        }
        // If there is no network, avoid showing the WebView default error page
        if (!NetworkUtils.isNetworkAvailable(requireContext())) {
            showError()
            return
        }
        showLoading()
        webView.loadUrl(active.url)
    }

    private fun showLoading() {
        progress.visibility = View.VISIBLE
        errorView.visibility = View.GONE
        webView.visibility = View.INVISIBLE
    }

    private fun hideLoading() {
        progress.visibility = View.GONE
        errorView.visibility = View.GONE
        webView.visibility = View.VISIBLE
    }

    private fun showError() {
        progress.visibility = View.GONE
        errorView.visibility = View.VISIBLE
        webView.visibility = View.GONE
    }
}
