/**
 * Client module to inject user profile into navbar
 * This replaces the static Sign In/Sign Up links with dynamic auth-aware links
 */
import React from "react";
import ExecutionEnvironment from "@docusaurus/ExecutionEnvironment";
import { createRoot } from "react-dom/client";
import {
  AuthProvider,
  useAuthContext,
} from "@site/src/components/Auth/AuthProvider";
import { Avatar } from "@site/src/components/Avatar/Avatar";
import Link from "@docusaurus/Link";
import useDocusaurusContext from "@docusaurus/useDocusaurusContext";

// Memoize component to prevent unnecessary re-renders
const UserProfileNavbarContent = React.memo(
  function UserProfileNavbarContent() {
    const { user, loading, refreshSession } = useAuthContext();
    const { siteConfig } = useDocusaurusContext();
    const baseUrl = siteConfig.baseUrl || "";
    const hasRefreshedRef = React.useRef(false);
    const mountedRef = React.useRef(true);

    console.log(
      "[UserProfileNavbarContent] 🔄 Render - user:",
      user?.email || "null",
      "loading:",
      loading
    );

    // Refresh session only once on mount - use ref to prevent multiple refreshes
    React.useEffect(() => {
      mountedRef.current = true;

      // Only refresh once, even if component re-mounts
      if (hasRefreshedRef.current) {
        console.log(
          "[UserProfileNavbarContent] ⏸️ Already refreshed, skipping"
        );
        return;
      }

      console.log(
        "[UserProfileNavbarContent] 🚀 Mounted, starting session refresh..."
      );
      hasRefreshedRef.current = true;

      // Immediate refresh (only once)
      refreshSession()
        .then(() => {
          if (mountedRef.current) {
            console.log(
              "[UserProfileNavbarContent] ✅ Initial refresh completed"
            );
          }
        })
        .catch((err) => {
          if (mountedRef.current) {
            console.error(
              "[UserProfileNavbarContent] ❌ Initial refresh failed:",
              err
            );
          }
        });

      // Single delayed check to catch late session availability
      const delayedCheck = setTimeout(() => {
        if (mountedRef.current) {
          console.log("[UserProfileNavbarContent] ⏰ Delayed check (1s)");
          refreshSession();
        }
      }, 1000);

      return () => {
        mountedRef.current = false;
        clearTimeout(delayedCheck);
      };
      // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []); // Empty deps - only run on mount

    // Listen for auth state change events - but debounce to prevent loops
    // Use refs to avoid recreating listeners when refreshSession changes
    const refreshSessionRef = React.useRef(refreshSession);
    const lastRefreshRef = React.useRef(0);
    const REFRESH_DEBOUNCE_MS = 2000; // Only refresh once per 2 seconds max

    // Update ref when refreshSession changes (but don't recreate listeners)
    React.useEffect(() => {
      refreshSessionRef.current = refreshSession;
    }, [refreshSession]);

    // Set up event listeners only once - they use refs so they always have latest function
    React.useEffect(() => {
      const handleAuthChange = () => {
        const now = Date.now();
        // Debounce: only refresh if it's been at least 2 seconds since last refresh
        if (now - lastRefreshRef.current > REFRESH_DEBOUNCE_MS) {
          console.log(
            "[UserProfileNavbarContent] Auth state change event received, refreshing session..."
          );
          lastRefreshRef.current = now;
          refreshSessionRef.current();
        } else {
          console.log(
            "[UserProfileNavbarContent] Auth state change ignored (debounced)"
          );
        }
      };
      window.addEventListener("auth-state-change", handleAuthChange);

      // Listen for storage changes (Better Auth uses localStorage) - also debounced
      const handleStorageChange = (e: StorageEvent) => {
        if (
          e.key?.includes("auth") ||
          e.key?.includes("session") ||
          e.key?.includes("better-auth")
        ) {
          const now = Date.now();
          if (now - lastRefreshRef.current > REFRESH_DEBOUNCE_MS) {
            console.log(
              "[UserProfileNavbarContent] Storage change detected, refreshing session..."
            );
            lastRefreshRef.current = now;
            refreshSessionRef.current();
          }
        }
      };
      window.addEventListener("storage", handleStorageChange);

      // Also listen for same-tab storage changes (using a custom event) - also debounced
      const handleLocalStorageChange = (e: Event) => {
        const customEvent = e as CustomEvent;
        if (
          customEvent.detail?.key?.includes("auth") ||
          customEvent.detail?.key?.includes("session")
        ) {
          const now = Date.now();
          if (now - lastRefreshRef.current > REFRESH_DEBOUNCE_MS) {
            console.log(
              "[UserProfileNavbarContent] Local storage change detected, refreshing session..."
            );
            lastRefreshRef.current = now;
            refreshSessionRef.current();
          }
        }
      };
      window.addEventListener("local-storage-change", handleLocalStorageChange);

      return () => {
        window.removeEventListener("auth-state-change", handleAuthChange);
        window.removeEventListener("storage", handleStorageChange);
        window.removeEventListener(
          "local-storage-change",
          handleLocalStorageChange
        );
      };
    }, []); // Empty deps - listeners don't need to be recreated, they use refs

    // Show loading state - but only for a brief moment
    if (loading) {
      console.log("[UserProfileNavbarContent] ⏳ Still loading...");
      // Show nothing while loading to avoid flicker
      return null;
    }

    // Show sign-up/sign-in buttons when not authenticated
    if (!user) {
      console.log(
        "[UserProfileNavbarContent] ❌ No user, showing Sign In/Sign Up buttons"
      );
      return (
        <>
          <a href={`${baseUrl}signin`} className="navbar__item navbar__link">
            Sign In
          </a>
          <a href={`${baseUrl}signup`} className="navbar__item navbar__link">
            Sign Up
          </a>
        </>
      );
    }

    // Show avatar with dropdown when authenticated
    console.log(
      "[UserProfileNavbarContent] ✅ User authenticated, showing Avatar. User:",
      user.email
    );
    return <Avatar />;
  },
  (prevProps, nextProps) => {
    // Custom comparison function - only re-render if user or loading actually changed
    // Since we're using context, this won't be called, but it's good practice
    return false; // Always re-render when context changes (which is what we want)
  }
);

// Docusaurus client modules execute immediately - they don't return components
// This code runs when the module is loaded
console.log("[NavbarInject] 🚀 NavbarInject module loaded!");

if (ExecutionEnvironment.canUseDOM) {
  console.log("[NavbarInject] ✅ DOM available, setting up injection...");

  // Execute immediately - don't wait for React
  (function setupNavbarInjection() {
    console.log("[NavbarInject] 📦 Starting injection setup...");
    let root: ReturnType<typeof createRoot> | null = null;
    let container: HTMLDivElement | null = null;

    // Helper function to remove static Sign In/Sign Up links
    const removeStaticLinks = (navbar: Element) => {
      const allItems = Array.from(navbar.children);
      let removedCount = 0;
      allItems.forEach((item: any) => {
        if (item.id === "user-profile-navbar-container") return;

        // Check text content
        const text =
          item.textContent?.trim() ||
          item.querySelector("a")?.textContent?.trim() ||
          "";
        // Check href attribute
        const href = item.querySelector("a")?.getAttribute("href") || "";
        const isSignIn =
          text === "Sign In" ||
          text.includes("Sign In") ||
          href.includes("/signin");
        const isSignUp =
          text === "Sign Up" ||
          text.includes("Sign Up") ||
          href.includes("/signup");

        if (isSignIn || isSignUp) {
          console.log("[NavbarInject] Removing static link:", text || href);
          item.remove();
          removedCount++;
        }
      });
      return removedCount > 0;
    };

    let isInjecting = false; // Flag to prevent concurrent injections
    let lastInjectTime = 0;
    const INJECT_DEBOUNCE_MS = 500; // Don't inject more than once per 500ms

    const injectUserProfile = () => {
      // CORE FIX: If container and root already exist, DO NOTHING
      // React will handle all updates automatically - calling this function again causes infinite loops
      if (container && root) {
        console.log(
          "[NavbarInject] ⏸️ Container and root already exist - React handles updates, skipping injection"
        );
        // Just remove any static links that might have reappeared
        const navbar = document.querySelector(".navbar__items--right");
        if (navbar) {
          removeStaticLinks(navbar);
        }
        return;
      }

      // Debounce: prevent rapid-fire injections
      const now = Date.now();
      if (now - lastInjectTime < INJECT_DEBOUNCE_MS) {
        console.log("[NavbarInject] ⏸️ Injection debounced");
        return;
      }

      // Prevent concurrent injections
      if (isInjecting) {
        console.log(
          "[NavbarInject] ⏸️ Injection already in progress, skipping"
        );
        return;
      }

      isInjecting = true;
      lastInjectTime = now;

      const navbar = document.querySelector(".navbar__items--right");
      if (!navbar) {
        isInjecting = false;
        // Retry after a short delay
        setTimeout(injectUserProfile, 100);
        return;
      }

      console.log("[NavbarInject] Injecting user profile into navbar...");

      // Always remove static links first, regardless of container state
      removeStaticLinks(navbar);

      // Check if container already exists
      const existingContainer = document.getElementById(
        "user-profile-navbar-container"
      );
      if (existingContainer) {
        container = existingContainer as HTMLDivElement;
        // Only create root if it doesn't exist (first time setup)
        if (!root) {
          console.log(
            "[NavbarInject] 🔄 Container exists but no root, creating root..."
          );
          root = createRoot(container);
          root.render(
            <AuthProvider>
              <UserProfileNavbarContent />
            </AuthProvider>
          );
        } else {
          // Container and root both exist - DO NOTHING
          // React will handle updates automatically through context
          // Calling root.render() again creates a new AuthProvider instance and causes infinite loops
          console.log(
            "[NavbarInject] ✅ Container and root exist, doing nothing (React handles updates)"
          );
        }
        // Remove any static links that might have reappeared
        removeStaticLinks(navbar);
        isInjecting = false;
        return;
      }

      // Find existing Sign In / Sign Up links
      const items = Array.from(navbar.children);
      const signInItem = items.find((el: any) => {
        const text =
          el.textContent?.trim() ||
          el.querySelector("a")?.textContent?.trim() ||
          "";
        const href = el.querySelector("a")?.getAttribute("href") || "";
        return (
          (text === "Sign In" ||
            text.includes("Sign In") ||
            href.includes("/signin")) &&
          el.id !== "user-profile-navbar-container"
        );
      });
      const signUpItem = items.find((el: any) => {
        const text =
          el.textContent?.trim() ||
          el.querySelector("a")?.textContent?.trim() ||
          "";
        const href = el.querySelector("a")?.getAttribute("href") || "";
        return (
          (text === "Sign Up" ||
            text.includes("Sign Up") ||
            href.includes("/signup")) &&
          el.id !== "user-profile-navbar-container"
        );
      });

      // Create container for user profile
      container = document.createElement("div");
      container.id = "user-profile-navbar-container";
      container.className = "navbar__item";
      container.style.display = "flex";
      container.style.gap = "0.5rem";
      container.style.alignItems = "center";

      // Replace Sign In with container or insert if not found
      if (signInItem && signInItem.id !== "user-profile-navbar-container") {
        signInItem.replaceWith(container);
      } else {
        // Insert at the end of navbar items (right side), but before GitHub link if it exists
        const githubItem = Array.from(navbar.children).find((el: any) => {
          const href = el.querySelector("a")?.getAttribute("href") || "";
          return href.includes("github.com");
        });
        if (githubItem) {
          navbar.insertBefore(container, githubItem);
        } else {
          navbar.appendChild(container);
        }
      }

      // Remove Sign Up if it's a separate item
      if (
        signUpItem &&
        signUpItem !== signInItem &&
        signUpItem.id !== "user-profile-navbar-container"
      ) {
        signUpItem.remove();
      }

      // Final cleanup - remove any remaining Sign In/Sign Up links
      removeStaticLinks(navbar);

      console.log(
        "[NavbarInject] 🎯 Creating container and rendering UserProfileNavbarContent..."
      );
      // Render user profile component ONCE - React will handle all future updates automatically
      // DO NOT call root.render() again after this - it creates new AuthProvider instances and causes infinite loops
      root = createRoot(container);
      root.render(
        <AuthProvider>
          <UserProfileNavbarContent />
        </AuthProvider>
      );
      console.log(
        "[NavbarInject] ✅ Component rendered to DOM - React will handle all future updates"
      );
      isInjecting = false;
    };

    // Initial injection - only run once, with a few retries if navbar isn't ready
    let injectionAttempts = 0;
    const maxInjectionAttempts = 3;

    const runInjection = () => {
      if (container && root) {
        console.log("[NavbarInject] ✅ Already injected, skipping");
        return;
      }

      injectionAttempts++;
      if (injectionAttempts <= maxInjectionAttempts) {
        injectUserProfile();
        // Only retry if container wasn't created
        if (!container) {
          setTimeout(runInjection, 500);
        }
      }
    };

    if (document.readyState === "loading") {
      document.addEventListener("DOMContentLoaded", runInjection);
    } else {
      runInjection();
    }

    // Handle Docusaurus navigation - ensure container persists across page changes
    let navigationCheckTimeout: ReturnType<typeof setTimeout> | null = null;
    const handleNavigation = () => {
      // Debounce navigation checks
      if (navigationCheckTimeout) {
        clearTimeout(navigationCheckTimeout);
      }
      navigationCheckTimeout = setTimeout(() => {
        const navbar = document.querySelector(".navbar__items--right");
        if (!navbar) return;

        // Check if container still exists in DOM
        const existingContainer = document.getElementById(
          "user-profile-navbar-container"
        );

        if (container && root && existingContainer) {
          // Container exists - just remove any static links that might have reappeared
          removeStaticLinks(navbar);
          // Ensure container is still in navbar (might have been moved)
          if (!navbar.contains(existingContainer)) {
            console.log(
              "[NavbarInject] Container exists but not in navbar, re-inserting..."
            );
            const githubItem = Array.from(navbar.children).find((el: any) => {
              const href = el.querySelector("a")?.getAttribute("href") || "";
              return href.includes("github.com");
            });
            if (githubItem) {
              navbar.insertBefore(existingContainer, githubItem);
            } else {
              navbar.appendChild(existingContainer);
            }
          }
        } else if (existingContainer && !root) {
          // Container exists but root was lost (navigation cleared React root)
          console.log(
            "[NavbarInject] Container exists but root lost, recreating root..."
          );
          container = existingContainer as HTMLDivElement;
          root = createRoot(container);
          root.render(
            <AuthProvider>
              <UserProfileNavbarContent />
            </AuthProvider>
          );
          removeStaticLinks(navbar);
        } else if (!existingContainer && !isInjecting) {
          // Container doesn't exist - try to inject once
          console.log(
            "[NavbarInject] Container missing after navigation, re-injecting..."
          );
          container = null;
          root = null;
          setTimeout(injectUserProfile, 100);
        }
      }, 150); // Small delay to let Docusaurus finish its navigation
    };

    // Use MutationObserver to watch for navbar changes (safer than intercepting history API)
    let observer: MutationObserver | null = null;
    const setupObserver = () => {
      const navbar = document.querySelector(".navbar__items--right");
      if (!navbar) return;

      if (observer) {
        observer.disconnect();
      }

      observer = new MutationObserver((mutations) => {
        // Check if navbar structure changed
        const hasRelevantChanges = mutations.some((mutation) => {
          return (
            mutation.type === "childList" &&
            (mutation.addedNodes.length > 0 || mutation.removedNodes.length > 0)
          );
        });

        if (hasRelevantChanges) {
          handleNavigation();
        }
      });

      observer.observe(navbar, {
        childList: true,
        subtree: false,
      });
    };

    // Set up observer after initial injection
    setTimeout(setupObserver, 1000);

    // Also listen for popstate (browser back/forward)
    window.addEventListener("popstate", handleNavigation);

    // Listen for storage events - but ONLY to remove static links, NOT to re-inject
    const handleStorageChange = () => {
      handleNavigation();
    };
    window.addEventListener("storage", handleStorageChange);

    // Listen for auth events - but ONLY to remove static links, NOT to re-inject
    const handleAuthChange = () => {
      handleNavigation();
    };
    window.addEventListener(
      "auth-state-change",
      handleAuthChange as EventListener
    );

    // Listen for focus events - but ONLY to remove static links, NOT to re-inject
    const handleFocus = () => {
      handleNavigation();
    };
    window.addEventListener("focus", handleFocus);

    // Poll for container persistence and static link removal
    const pollInterval = setInterval(() => {
      handleNavigation();
    }, 5000); // Check every 5 seconds

    // Cleanup function
    const cleanup = () => {
      if (navigationCheckTimeout) {
        clearTimeout(navigationCheckTimeout);
      }
      if (observer) {
        observer.disconnect();
      }
      window.removeEventListener("popstate", handleNavigation);
      window.removeEventListener("storage", handleStorageChange);
      window.removeEventListener(
        "auth-state-change",
        handleAuthChange as EventListener
      );
      window.removeEventListener("focus", handleFocus);
      clearInterval(pollInterval);
      if (root && container) {
        root.unmount();
      }
    };

    // Return cleanup function (will be called if module is unloaded)
    return cleanup;
  })();
} else {
  console.log("[NavbarInject] ⚠️ DOM not available, skipping injection");
}
