/*
 * Copyright (c) 2026 HPMicro
 * SPDX-License-Identifier: Apache-2.0
 */

(function () {
    "use strict";

    const storageKey = "sdk-glue-sidebar-expanded-v3";
    const windowNamePrefix = storageKey + ":";
    const urlStateKey = "sidebar-state";

    function directChild(element, tagName) {
        return Array.from(element.children).find(
            (child) => child.tagName === tagName
        );
    }

    function itemKey(item) {
        const link = directChild(item, "A");

        if (!link) {
            return null;
        }

        const url = new URL(link.href, document.baseURI);
        return url.pathname + url.hash;
    }

    function readExpandedItems() {
        let serialized = null;

        if (location.protocol === "file:") {
            serialized = new URL(location.href).searchParams.get(urlStateKey);
        }

        if (!serialized && location.protocol === "file:" &&
            window.name.startsWith(windowNamePrefix)) {
            serialized = window.name.slice(windowNamePrefix.length);
        }

        try {
            serialized = serialized || localStorage.getItem(storageKey) || "[]";
            return new Set(JSON.parse(serialized));
        } catch (error) {
            return new Set();
        }
    }

    function saveExpandedItems() {
        const expandedItems = [];

        document.querySelectorAll(".wy-menu-vertical li.current").forEach(
            (item) => {
                if (!directChild(item, "UL")) {
                    return;
                }

                const key = itemKey(item);
                if (key) {
                    expandedItems.push(key);
                }
            }
        );

        const serialized = JSON.stringify(expandedItems);

        try {
            localStorage.setItem(storageKey, serialized);
        } catch (error) {
            /* Browsing generated files can disable local storage. */
        }

        if (location.protocol === "file:") {
            window.name = windowNamePrefix + serialized;
        }

        return serialized;
    }

    function restoreExpandedItems() {
        const expandedItems = readExpandedItems();

        document.querySelectorAll(".wy-menu-vertical li").forEach((item) => {
            if (!directChild(item, "UL") || !expandedItems.has(itemKey(item))) {
                return;
            }

            item.classList.add("current");
            item.setAttribute("aria-expanded", "true");
        });
    }

    let installed = false;

    function installSidebarState() {
        const navigation = window.SphinxRtdTheme &&
            window.SphinxRtdTheme.Navigation;
        const jquery = window.jQuery;

        if (installed || !navigation || !jquery) {
            return;
        }

        installed = true;

        navigation.toggleCurrent = function (link) {
            const item = link.closest("li");
            const element = item.get(0);

            if (!element || !directChild(element, "UL")) {
                return;
            }

            const expanded = item.hasClass("current");
            item.toggleClass("current", !expanded);
            item.attr("aria-expanded", expanded ? "false" : "true");
            saveExpandedItems();
        };

        restoreExpandedItems();
        saveExpandedItems();

        document.addEventListener("click", function (event) {
            if (location.protocol !== "file:" ||
                event.target.closest("button.toctree-expand")) {
                return;
            }

            const link = event.target.closest("a[href]");
            if (!link || link.target || link.hasAttribute("download")) {
                return;
            }

            const target = new URL(link.href, document.baseURI);
            if (target.protocol !== "file:") {
                return;
            }

            target.searchParams.set(urlStateKey, saveExpandedItems());
            link.href = target.href;
        }, true);

        window.addEventListener("pagehide", saveExpandedItems);
        window.addEventListener("hashchange", function () {
            window.setTimeout(restoreExpandedItems, 0);
        });
    }

    function installWhenThemeIsReady() {
        const navigation = window.SphinxRtdTheme &&
            window.SphinxRtdTheme.Navigation;

        if (!navigation || !navigation.isRunning || !navigation.navBar) {
            window.setTimeout(installWhenThemeIsReady, 10);
            return;
        }

        window.setTimeout(installSidebarState, 0);
    }

    window.addEventListener("load", installWhenThemeIsReady);
}());
