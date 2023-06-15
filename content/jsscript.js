
const App = {
    data() {
        return {
            contributor_avatar_link: null,
            activeIndex: '1'
        };
    },
    methods: {
        getGuidePage() {
            console.log("getGuidePage");
            // axios.get('/guide');
            // Make a request for a user with a given ID
            window.location.replace("/guide");
            // axios.get('/guide')
            // 	.then(function (response) {
            // 		// handle success
            // 		console.log(response);
            // 	})
            // 	.catch(function (error) {
            // 		// handle error
            // 		console.log(error);
            // 	})
            // 	.finally(function () {
            // 		// always executed
            // 	})
        },


        getControlPage() {
            window.location.replace("/");
            // axios.get('/guide')
            // 	.then(function (response) {console.log(response); });
            console.log("getControlPage");
            // axios.get('/');
        }
    },

    mounted: function () {
        var self = this;
        axios.get("/api/v1/contributor_avatar_links").then(function (res) {
            self.contributor_avatar_link = res.data;
        });
    }
};
const app = Vue.createApp(App);
app.use(ElementPlus);
app.mount("#app");
hljs.highlightAll();
