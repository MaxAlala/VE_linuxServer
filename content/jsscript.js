
function setOfflineOrOnline(isOnl) {
    id = "isOnline";
    var el = document.getElementById(id);

    if (isOnl) {
        el.style.color = "rgb(76, 212, 22)";
        el.innerHTML = "Online";
    }
    else {
        el.style.color = "red";
        el.innerHTML = "Offline";
    }

};

const App = {
    data() {
        return {
            contributor_avatar_link: null,
            activeIndex: '1',
            isPatrolOn: false,
            timeOfBeingOnline: "0",
            timeOfStartUtc: "0:0:0 0:0:0+0"
        };
    },
    methods: {

        togglePatrol() {
            axios.post('/togglePatrol?togglePatrol=' + !isPatrolOn)
                .then(function (response) {
                    console.log(response);
                    console.log("/togglePatrol?togglePatrol=" + !isPatrolOn);

                })
                .catch(function (error) {
                    console.log(error);
                });
        },

        getTimeOfBeingOnline() {
            // console.log(`Hello world!`);
            axios.get('/getTimeOfBeingOnline')
                .then(function (response) {
                    // handle success
                    // console.log(response);
                    timeOfBeingOnline = response.data['timeOfBeingOnline'];
                    // console.log('timeOfBeingOnline=' + timeOfBeingOnline);

                    let id = "timeOfBeingOnline";
                    // function changeValue(id, newText) {
                    var el = document.getElementById(id);

                    // if (isPatrolOn == true)
                    el.innerHTML = "Online in min:" + timeOfBeingOnline;  // change the displayed text on the screen
                    // else if(isPatrolOn == false)
                    // el.innerHTML = "start patroling";
                    setOfflineOrOnline(true);

                    // el.value     = newText;  // change the value passed to the next page

                    // return false;
                    // }
                })
                .catch(function (error) {
                    setOfflineOrOnline(false);
                    // handle error
                    console.log(error);
                })
                .finally(function () {
                    // setOfflineOrOnline(false);
                    // always executed
                });
        },

        getTimeOfStartUtc() {
            // console.log(`Hello world!`);
            axios.get('/getTimeOfStart')
                .then(function (response) {
                    // handle success
                    // console.log(response);
                    timeOfStartUtc = response.data['timeOfStartUtc'];
                    // console.log('timeOfStartUtc=' + timeOfStartUtc);

                    let id = "timeOfStartUtc";
                    // function changeValue(id, newText) {
                    var el = document.getElementById(id);

                    // if (isPatrolOn == true)
                    el.innerHTML = "startedAt:" + timeOfStartUtc;  // change the displayed text on the screen
                    // else if(isPatrolOn == false)
                    // el.innerHTML = "start patroling";

                    // el.value     = newText;  // change the value passed to the next page

                    // return false;
                    // }
                })
                .catch(function (error) {
                    // handle error
                    console.log(error);
                })
                .finally(function () {
                    // always executed
                });
        },

        getCurrentPatrol() {
            // console.log(`Hello world!`);
            axios.get('/getPatrol')
                .then(function (response) {
                    // handle success
                    console.log(response);
                    isPatrolOn = response.data['isPatrolOn'];
                    console.log('isPatrolOn=' + isPatrolOn);

                    let id = "patrolButton";
                    // function changeValue(id, newText) {
                    var el = document.getElementById(id);

                    if (isPatrolOn == true)
                        el.innerHTML = "stop patroling";  // change the displayed text on the screen
                    else if (isPatrolOn == false)
                        el.innerHTML = "start patroling";

                    // el.value     = newText;  // change the value passed to the next page

                    // return false;
                    // }
                })
                .catch(function (error) {
                    // handle error
                    console.log(error);
                })
                .finally(function () {
                    // always executed
                });
        },
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
        // App.methods.getTimeOfStartUtc();
    }
};
const app = Vue.createApp(App);
app.use(ElementPlus);
app.mount("#app");

// const myFunction = () => {
//     App.methods.getCurrentPatrol();
//     console.log(`Hello world!`);
// };

setInterval(App.methods.getCurrentPatrol, 1000);
setInterval(App.methods.getTimeOfBeingOnline, 1000);
setInterval(App.methods.getTimeOfStartUtc, 1000);

hljs.highlightAll();
