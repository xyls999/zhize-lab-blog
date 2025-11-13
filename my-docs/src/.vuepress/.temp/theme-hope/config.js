import { Layout, NotFound, injectDarkMode, setupDarkMode, setupSidebarItems, scrollPromise } from "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/vuepress-theme-hope@2.0.0-r_5f98ee969002387c1f5b6b91f9256603/node_modules/vuepress-theme-hope/lib/bundle/exports/base.js";

import { defineCatalogInfoGetter } from "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/@vuepress+plugin-catalog@2._7c7b2e8eaef99cb6ba270078e87ac445/node_modules/@vuepress/plugin-catalog/lib/client/index.js"
import { h } from "vue"
import { resolveComponent } from "vue"
import { GlobalEncrypt, LocalEncrypt } from "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/vuepress-theme-hope@2.0.0-r_5f98ee969002387c1f5b6b91f9256603/node_modules/vuepress-theme-hope/lib/bundle/exports/encrypt.js";
import "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/vuepress-theme-hope@2.0.0-r_5f98ee969002387c1f5b6b91f9256603/node_modules/vuepress-theme-hope/lib/bundle/styles/encrypt/bundle.scss"

import "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/@vuepress+helper@2.0.0-rc.1_cff9eb5b88fb6a429afb371eb704a643/node_modules/@vuepress/helper/lib/client/styles/colors.css";
import "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/@vuepress+helper@2.0.0-rc.1_cff9eb5b88fb6a429afb371eb704a643/node_modules/@vuepress/helper/lib/client/styles/normalize.css";
import "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/@vuepress+helper@2.0.0-rc.1_cff9eb5b88fb6a429afb371eb704a643/node_modules/@vuepress/helper/lib/client/styles/sr-only.css";
import "E:/lab1/zhize-web-Code/my-docs/node_modules/.pnpm/vuepress-theme-hope@2.0.0-r_5f98ee969002387c1f5b6b91f9256603/node_modules/vuepress-theme-hope/lib/bundle/styles/bundle.scss";

defineCatalogInfoGetter((meta) => {
  const title = meta.title;
  const shouldIndex = meta.index ?? true;
  const icon = meta.icon;

  return shouldIndex ? {
    title,
    content: icon ? () =>[h(resolveComponent("VPIcon"), { icon, sizing: "both" }), title] : null,
    order: meta.order,
    index: meta.index,
  } : null;
});

export default {
  enhance: ({ app, router }) => {
    const { scrollBehavior } = router.options;

    router.options.scrollBehavior = async (...args) => {
      await scrollPromise.wait();

      return scrollBehavior(...args);
    };

    // inject global properties
    injectDarkMode(app);

    app.component("GlobalEncrypt", GlobalEncrypt);
    app.component("LocalEncrypt", LocalEncrypt);
  },
  setup: () => {
    setupDarkMode();
    setupSidebarItems();

  },
  layouts: {
    Layout,
    NotFound,

  }
};
