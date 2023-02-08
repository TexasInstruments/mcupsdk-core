/*
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "utils/mbedtls_test_utils.h"

/* ========================================================================== */
/*                           TEST-DATA DECLARATION                            */
/* ========================================================================== */

/*
    data_sha is an array of string literals of test data for mbedTLS sanity tests for SHA
    these data vectors will be used in case of both software and hardware cryptography offloaded mbedTLS (TO DO)
*/

const char* data_sha[] = {
    "depends_on:0",
    "0:hex:\"\":hex:\"da39a3ee5e6b4b0d3255bfef95601890afd80709\"",
    "depends_on:0",
    "0:hex:\"a8\":hex:\"99f2aa95e36f95c2acb0eaf23998f030638f3f15\"",
    "depends_on:0",
    "0:hex:\"3000\":hex:\"f944dcd635f9801f7ac90a407fbc479964dec024\"",
    "depends_on:0",
    "0:hex:\"42749e\":hex:\"a444319e9b6cc1e8464c511ec0969c37d6bb2619\"",
    "depends_on:0",
    "0:hex:\"9fc3fe08\":hex:\"16a0ff84fcc156fd5d3ca3a744f20a232d172253\"",
    "depends_on:0",
    "0:hex:\"b5c1c6f1af\":hex:\"fec9deebfcdedaf66dda525e1be43597a73a1f93\"",
    "depends_on:0",
    "0:hex:\"ec29561244ede706b6eb30a1c371d74450a105c3f9735f7fa9fe38cf67f304a5736a106e92e17139a6813b1c81a4f3d3fb9546ab4296fa9f722826c066869edacd73b2548035185813e22634a9da44000d95a281ff9f264ecce0a931222162d021cca28db5f3c2aa24945ab1e31cb413ae29810fd794cad5dfaf29ec43cb38d198fe4ae1da2359780221405bd6712a5305da4b1b737fce7cd21c0eb7728d08235a9011\":hex:\"970111c4e77bcc88cc20459c02b69b4aa8f58217\"",
    "depends_on:0",
    "0:hex:\"5fc2c3f6a7e79dc94be526e5166a238899d54927ce470018fbfd668fd9dd97cbf64e2c91584d01da63be3cc9fdff8adfefc3ac728e1e335b9cdc87f069172e323d094b47fa1e652afe4d6aa147a9f46fda33cacb65f3aa12234746b9007a8c85fe982afed7815221e43dba553d8fe8a022cdac1b99eeeea359e5a9d2e72e382dffa6d19f359f4f27dc3434cd27daeeda8e38594873398678065fbb23665aba9309d946135da0e4a4afdadff14db18e85e71dd93c3bf9faf7f25c8194c4269b1ee3d9934097ab990025d9c3aaf63d5109f52335dd3959d38ae485050e4bbb6235574fc0102be8f7a306d6e8de6ba6becf80f37415b57f9898a5824e77414197422be3d36a6080\":hex:\"0423dc76a8791107d14e13f5265b343f24cc0f19\"",
    "depends_on:0",
    "0:hex:\"0f865f46a8f3aed2da18482aa09a8f390dc9da07d51d1bd10fe0bf5f3928d5927d08733d32075535a6d1c8ac1b2dc6ba0f2f633dc1af68e3f0fa3d85e6c60cb7b56c239dc1519a007ea536a07b518ecca02a6c31b46b76f021620ef3fc6976804018380e5ab9c558ebfc5cb1c9ed2d974722bf8ab6398f1f2b82fa5083f85c16a5767a3a07271d67743f00850ce8ec428c7f22f1cf01f99895c0c844845b06a06cecb0c6cf83eb55a1d4ebc44c2c13f6f7aa5e0e08abfd84e7864279057abc471ee4a45dbbb5774afa24e51791a0eada11093b88681fe30baa3b2e94113dc63342c51ca5d1a6096d0897b626e42cb91761058008f746f35465465540ad8c6b8b60f7e1461b3ce9e6529625984cb8c7d46f07f735be067588a0117f23e34ff57800e2bbe9a1605fde6087fb15d22c5d3ac47566b8c448b0cee40373e5ba6eaa21abee71366afbb27dbbd300477d70c371e7b8963812f5ed4fb784fb2f3bd1d3afe883cdd47ef32beaea\":hex:\"6692a71d73e00f27df976bc56df4970650d90e45\"",
    "depends_on:0",
    "0:hex:\"8236153781bd2f1b81ffe0def1beb46f5a70191142926651503f1b3bb1016acdb9e7f7acced8dd168226f118ff664a01a8800116fd023587bfba52a2558393476f5fc69ce9c65001f23e70476d2cc81c97ea19caeb194e224339bcb23f77a83feac5096f9b3090c51a6ee6d204b735aa71d7e996d380b80822e4dfd43683af9c7442498cacbea64842dfda238cb099927c6efae07fdf7b23a4e4456e0152b24853fe0d5de4179974b2b9d4a1cdbefcbc01d8d311b5dda059136176ea698ab82acf20dd490be47130b1235cb48f8a6710473cfc923e222d94b582f9ae36d4ca2a32d141b8e8cc36638845fbc499bce17698c3fecae2572dbbd470552430d7ef30c238c2124478f1f780483839b4fb73d63a9460206824a5b6b65315b21e3c2f24c97ee7c0e78faad3df549c7ca8ef241876d9aafe9a309f6da352bec2caaa92ee8dca392899ba67dfed90aef33d41fc2494b765cb3e2422c8e595dabbfaca217757453fb322a13203f425f6073a9903e2dc5818ee1da737afc345f0057744e3a56e1681c949eb12273a3bfc20699e423b96e44bd1ff62e50a848a890809bfe1611c6787d3d741103308f849a790f9c015098286dbacfc34c1718b2c2b77e32194a75dda37954a320fa68764027852855a7e5b5274eb1e2cbcd27161d98b59ad245822015f48af82a45c0ed59be94f9af03d9736048570d6e3ef63b1770bc98dfb77de84b1bb1708d872b625d9ab9b06c18e5dbbf34399391f0f8aa26ec0dac7ff4cb8ec97b52bcb942fa6db2385dcd1b3b9d567aaeb425d567b0ebe267235651a1ed9bf78fd93d3c1dd077fe340bb04b00529c58f45124b717c168d07e9826e33376988bc5cf62845c2009980a4dfa69fbc7e5a0b1bb20a5958ca967aec68eb31dd8fccca9afcd30a26bab26279f1bf6724ff\":hex:\"11863b483809ef88413ca9b0084ac4a5390640af\"",
    "depends_on:1",
    "1:hex:\"\":hex:\"d14a028c2a3a2bc9476102bb288234c415a2b01f828ea62ac5b3e42f\"",
    "depends_on:1",
    "1:hex:\"ff\":hex:\"e33f9d75e6ae1369dbabf81b96b4591ae46bba30b591a6b6c62542b5\"",
    "depends_on:1",
    "1:hex:\"984c\":hex:\"2fa9df9157d9e027cfbc4c6a9df32e1adc0cbe2328ec2a63c5ae934e\"",
    "depends_on:1",
    "1:hex:\"50efd0\":hex:\"b5a9820413c2bf8211fbbf5df1337043b32fa4eafaf61a0c8e9ccede\"",
    "depends_on:1",
    "1:hex:\"e5e09924\":hex:\"fd19e74690d291467ce59f077df311638f1c3a46e510d0e49a67062d\"",
    "depends_on:1",
    "1:hex:\"21ebecb914\":hex:\"78f4a71c21c694499ce1c7866611b14ace70d905012c356323c7c713\"",
    "depends_on:1",
    "1:hex:\"fc488947c1a7a589726b15436b4f3d9556262f98fc6422fc5cdf20f0fad7fe427a3491c86d101ffe6b7514f06268f65b2d269b0f69ad9a97847eff1c16a2438775eb7be6847ccf11cb8b2e8dcd6640b095b49c0693fe3cf4a66e2d9b7ad68bff14f3ad69abf49d0aba36cbe0535202deb6599a47225ef05beb351335cd7bc0f480d691198c7e71305ffd53b39d33242bb79cfd98bfd69e137b5d18b2b89ac9ace01c8dbdcf2533cce3682ecc52118de0c1062ec2126c2e657d6ea3d9e2398e705d4b0b1f1ceecb266dffc4f31bf42744fb1e938dc22a889919ee1e73f463f7871fed720519e32186264b7ef2a0e5d9a18e6c95c0781894f77967f048951dec3b4d892a38710b1e3436d3c29088eb8b3da1789c25db3d3bc6c26081206e7155d210a89b80ca6ea877c41ff9947c0f25625dcb118294a163501f6239c326661a958fd12da4cd15a899f8b88cc723589056eaec5aa04a4cf5dbb6f480f9660423ccf38c486e210707e0fb25e1f126ceb2616f63e147a647dab0af9ebe89d65458bf636154a46e4cab95f5ee62da2c7974cd14b90d3e4f99f81733e85b3c1d5da2b508d9b90f5eed7eff0d9c7649de62bee00375454fee4a39576a5bbfdae428e7f8097bdf7797f167686cb68407e49079e4611ff3402b6384ba7b7e522bd2bb11ce8fd02ea4c1604d163ac4f6dde50b8b1f593f7edaadeac0868ed97df690200680c25f0f5d85431a529e4f339089dcdeda105e4ee51dead704cdf5a605c55fb055c9b0e86b8ba1b564c0dea3eb790a595cb103cb292268b07c5e59371e1a7ef597cd4b22977a820694c9f9aeb55d9de3ef62b75d6e656e3336698d960a3787bf8cf5b926a7faeef52ae128bcb5dc9e66d94b016c7b8e034879171a2d91c381f57e6a815b63b5ee6a6d2ff435b49f14c963966960194430d78f8f87627a67757fb3532b289550894da6dce4817a4e07f4d56877a1102ffcc8befa5c9f8fca6a4574d93ff70376c8861e0f8108cf907fce77ecb49728f86f034f80224b9695682e0824462f76cdb1fd1af151337b0d85419047a7aa284791718a4860cd586f7824b95bc837b6fd4f9be5aade68456e20356aa4d943dac36bf8b67b9e8f9d01a00fcda74b798bafa746c661b010f75b59904b29d0c8041504811c4065f82cf2ead58d2f595cbd8bc3e7043f4d94577b373b7cfe16a36fe564f505c03b70cfeb5e5f411c79481338aa67e86b3f5a2e77c21e454c333ae3da943ab723ab5f4c940395319534a5575f64acba0d0ecc43f60221ed3badf7289c9b3a7b903a2d6c94e15fa4c310dc4fa7faa0c24f405160a1002dbef20e4105d481db982f7243f79400a6e4cd9753c4b9732a47575f504b20c328fe9add7f432a4f075829da07b53b695037dc51737d3cd731934df333cd1a53fcf65aa31baa450ca501a6fae26e322347e618c5a444d92e9fec5a8261ae38b98fee5be77c02cec09ddccd5b3de92036\":hex:\"1302149d1e197c41813b054c942329d420e366530f5517b470e964fe\"",
    "depends_on:1",
    "2:hex:\"\":hex:\"e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855\"",
    "depends_on:1",
    "2:hex:\"bd\":hex:\"68325720aabd7c82f30f554b313d0570c95accbb7dc4b5aae11204c08ffe732b\"",
    "depends_on:1",
    "2:hex:\"5fd4\":hex:\"7c4fbf484498d21b487b9d61de8914b2eadaf2698712936d47c3ada2558f6788\"",
    "depends_on:1",
    "2:hex:\"b0bd69\":hex:\"4096804221093ddccfbf46831490ea63e9e99414858f8d75ff7f642c7ca61803\"",
    "depends_on:1",
    "2:hex:\"c98c8e55\":hex:\"7abc22c0ae5af26ce93dbb94433a0e0b2e119d014f8e7f65bd56c61ccccd9504\"",
    "depends_on:1",
    "2:hex:\"81a723d966\":hex:\"7516fb8bb11350df2bf386bc3c33bd0f52cb4c67c6e4745e0488e62c2aea2605\"",
    "depends_on:1",
    "2:hex:\"8390cf0be07661cc7669aac54ce09a37733a629d45f5d983ef201f9b2d13800e555d9b1097fec3b783d7a50dcb5e2b644b96a1e9463f177cf34906bf388f366db5c2deee04a30e283f764a97c3b377a034fefc22c259214faa99babaff160ab0aaa7e2ccb0ce09c6b32fe08cbc474694375aba703fadbfa31cf685b30a11c57f3cf4edd321e57d3ae6ebb1133c8260e75b9224fa47a2bb205249add2e2e62f817491482ae152322be0900355cdcc8d42a98f82e961a0dc6f537b7b410eff105f59673bfb787bf042aa071f7af68d944d27371c64160fe9382772372516c230c1f45c0d6b6cca7f274b394da9402d3eafdf733994ec58ab22d71829a98399574d4b5908a447a5a681cb0dd50a31145311d92c22a16de1ead66a5499f2dceb4cae694772ce90762ef8336afec653aa9b1a1c4820b221136dfce80dce2ba920d88a530c9410d0a4e0358a3a11052e58dd73b0b179ef8f56fe3b5a2d117a73a0c38a1392b6938e9782e0d86456ee4884e3c39d4d75813f13633bc79baa07c0d2d555afbf207f52b7dca126d015aa2b9873b3eb065e90b9b065a5373fe1fb1b20d594327d19fba56cb81e7b6696605ffa56eba3c27a438697cc21b201fd7e09f18deea1b3ea2f0d1edc02df0e20396a145412cd6b13c32d2e605641c948b714aec30c0649dc44143511f35ab0fd5dd64c34d06fe86f3836dfe9edeb7f08cfc3bd40956826356242191f99f53473f32b0cc0cf9321d6c92a112e8db90b86ee9e87cc32d0343db01e32ce9eb782cb24efbbbeb440fe929e8f2bf8dfb1550a3a2e742e8b455a3e5730e9e6a7a9824d17acc0f72a7f67eae0f0970f8bde46dcdefaed3047cf807e7f00a42e5fd11d40f5e98533d7574425b7d2bc3b3845c443008b58980e768e464e17cc6f6b3939eee52f713963d07d8c4abf02448ef0b889c9671e2f8a436ddeeffcca7176e9bf9d1005ecd377f2fa67c23ed1f137e60bf46018a8bd613d038e883704fc26e798969df35ec7bbc6a4fe46d8910bd82fa3cded265d0a3b6d399e4251e4d8233daa21b5812fded6536198ff13aa5a1cd46a5b9a17a4ddc1d9f85544d1d1cc16f3df858038c8e071a11a7e157a85a6a8dc47e88d75e7009a8b26fdb73f33a2a70f1e0c259f8f9533b9b8f9af9288b7274f21baeec78d396f8bacdcc22471207d9b4efccd3fedc5c5a2214ff5e51c553f35e21ae696fe51e8df733a8e06f50f419e599e9f9e4b37ce643fc810faaa47989771509d69a110ac916261427026369a21263ac4460fb4f708f8ae28599856db7cb6a43ac8e03d64a9609807e76c5f312b9d1863bfa304e8953647648b4f4ab0ed995e\":hex:\"4109cdbec3240ad74cc6c37f39300f70fede16e21efc77f7865998714aad0b5e\"",
    "depends_on:2",
    "3:hex:\"\":hex:\"38b060a751ac96384cd9327eb1b1e36a21fdb71114be07434c0cc7bf63f6e1da274edebfe76f65fbd51ad2f14898b95b\"",
    "depends_on:2",
    "3:hex:\"ab\":hex:\"fb94d5be118865f6fcbc978b825da82cff188faec2f66cb84b2537d74b4938469854b0ca89e66fa2e182834736629f3d\"",
    "depends_on:2",
    "3:hex:\"7c27\":hex:\"3d80be467df86d63abb9ea1d3f9cb39cd19890e7f2c53a6200bedc5006842b35e820dc4e0ca90ca9b97ab23ef07080fc\"",
    "depends_on:2",
    "3:hex:\"31f5ca\":hex:\"78d54b943421fdf7ba90a7fb9637c2073aa480454bd841d39ff72f4511fc21fb67797b652c0c823229342873d3bef955\"",
    "depends_on:2",
    "3:hex:\"7bdee3f8\":hex:\"8bdafba0777ee446c3431c2d7b1fbb631089f71d2ca417abc1d230e1aba64ec2f1c187474a6f4077d372c14ad407f99a\"",
    "depends_on:2",
    "3:hex:\"8f05604915\":hex:\"504e414bf1db1060f14c8c799e25b1e0c4dcf1504ebbd129998f0ae283e6de86e0d3c7e879c73ec3b1836c3ee89c2649\"",
    "depends_on:2",
    "3:hex:\"665da6eda214\":hex:\"4c022f112010908848312f8b8f1072625fd5c105399d562ea1d56130619a7eac8dfc3748fd05ee37e4b690be9daa9980\"",
    "depends_on:2",
    "3:hex:\"7f46ce506d593c4ed53c82edeb602037e0485befbee03f7f930fe532d18ff2a3f5fd6076672c8145a1bf40dd94f7abab47c9ae71c234213d2ad1069c2dac0b0ba15257ae672b8245960ae55bd50315c0097daa3a318745788d70d14706910809ca6e396237fe4934fa46f9ce782d66606d8bd6b2d283b1160513ce9c24e9f084b97891f99d4cdefc169a029e431ca772ba1bba426fce6f01d8e286014e5acc66b799e4db62bd4783322f8a32ff78e0de3957df50ce10871f4e0680df4e8ca3960af9bc6f4efa8eb3962d18f474eb178c3265cc46b8f2ff5ab1a7449fea297dfcfabfa01f28abbb7289bb354b691b5664ec6d098af51be19947ec5ba7ebd66380d1141953ba78d4aa5401679fa7b0a44db1981f864d3535c45afe4c61183d5b0ad51fae71ca07e34240283959f7530a32c70d95a088e501c230059f333b0670825009e7e22103ef22935830df1fac8ef877f5f3426dd54f7d1128dd871ad9a7d088f94c0e8712013295b8d69ae7623b880978c2d3c6ad26dc478f8dc47f5c0adcc618665dc3dc205a9071b2f2191e16cac5bd89bb59148fc719633752303aa08e518dbc389f0a5482caaa4c507b8729a6f3edd061efb39026cecc6399f51971cf7381d605e144a5928c8c2d1ad7467b05da2f202f4f3234e1aff19a0198a28685721c3d2d52311c721e3fdcbaf30214cdc3acff8c433880e104fb63f2df7ce69a97857819ba7ac00ac8eae1969764fde8f68cf8e0916d7e0c151147d4944f99f42ae50f30e1c79a42d2b6c5188d133d3cbbf69094027b354b295ccd0f7dc5a87d73638bd98ebfb00383ca0fa69cb8dcb35a12510e5e07ad8789047d0b63841a1bb928737e8b0a0c33254f47aa8bfbe3341a09c2b76dbcefa67e30df300d34f7b8465c4f869e51b6bcfe6cf68b238359a645036bf7f63f02924e087ce7457e483b6025a859903cb484574aa3b12cf946f32127d537c33bee3141b5db96d10a148c50ae045f287210757710d6846e04b202f79e87dd9a56bc6da15f84a77a7f63935e1dee00309cd276a8e7176cb04da6bb0e9009534438732cb42d008008853d38d19beba46e61006e30f7efd1bc7c2906b024e4ff898a1b58c448d68b43c6ab63f34f85b3ac6aa4475867e51b583844cb23829f4b30f4bdd817d88e2ef3e7b4fc0a624395b05ec5e8686082b24d29fef2b0d3c29e031d5f94f504b1d3df9361eb5ffbadb242e66c39a8094cfe62f85f639f3fd65fc8ae0c74a8f4c6e1d070b9183a434c722caaa0225f8bcd68614d6f0738ed62f8484ec96077d155c08e26c46be262a73e3551698bd70d8d5610cf37c4c306eed04ba6a040a9c3e6d7e15e8acda17f477c2484cf5c56b813313927be8387b1024f995e98fc87f1029091c01424bdc2b296c2eadb7d25b3e762a2fd0c2dcd1727ddf91db97c5984305265f3695a7f5472f2d72c94d68c27914f14f82aa8dd5fe4e2348b0ca967a3f98626a091552f5d0ffa2bf10350d23c996256c01fdeffb2c2c612519869f877e4929c6e95ff15040f1485e22ed14119880232fef3b57b3848f15b1766a5552879df8f06\":hex:\"cba9e3eb12a6f83db11e8a6ff40d1049854ee094416bc527fea931d8585428a8ed6242ce81f6769b36e2123a5c23483e\"",
    "depends_on:2",
    "4:hex:\"\":hex:\"cf83e1357eefb8bdf1542850d66d8007d620e4050b5715dc83f4a921d36ce9ce47d0d13c5d85f2b0ff8318d2877eec2f63b931bd47417a81a538327af927da3e\"",
    "depends_on:2",
    "4:hex:\"8f\":hex:\"e4cd2d19931b5aad9c920f45f56f6ce34e3d38c6d319a6e11d0588ab8b838576d6ce6d68eea7c830de66e2bd96458bfa7aafbcbec981d4ed040498c3dd95f22a\"",
    "depends_on:2",
    "4:hex:\"e724\":hex:\"7dbb520221a70287b23dbcf62bfc1b73136d858e86266732a7fffa875ecaa2c1b8f673b5c065d360c563a7b9539349f5f59bef8c0c593f9587e3cd50bb26a231\"",
    "depends_on:2",
    "4:hex:\"de4c90\":hex:\"33ce98281045a5c4c9df0363d8196f1d7dfcd5ee46ac89776fd8a4344c12f123a66788af5bd41ceff1941aa5637654b4064c88c14e00465ab79a2fc6c97e1014\"",
    "depends_on:2",
    "4:hex:\"a801e94b\":hex:\"dadb1b5a27f9fece8d86adb2a51879beb1787ff28f4e8ce162cad7fee0f942efcabbf738bc6f797fc7cc79a3a75048cd4c82ca0757a324695bfb19a557e56e2f\"",
    "depends_on:2",
    "4:hex:\"94390d3502\":hex:\"b6175c4c4cccf69e0ce5f0312010886ea6b34d43673f942ae42483f9cbb7da817de4e11b5d58e25a3d9bd721a22cdffe1c40411cc45df1911fa5506129b69297\"",
    "depends_on:2",
    "4:hex:\"49297dd63e5f\":hex:\"1fcc1e6f6870859d11649f5e5336a9cd16329c029baf04d5a6edf257889a2e9522b497dd656bb402da461307c4ee382e2e89380c8e6e6e7697f1e439f650fa94\"",
    "depends_on:2",
    "4:hex:\"990d1ae71a62d7bda9bfdaa1762a68d296eee72a4cd946f287a898fbabc002ea941fd8d4d991030b4d27a637cce501a834bb95eab1b7889a3e784c7968e67cbf552006b206b68f76d9191327524fcc251aeb56af483d10b4e0c6c5e599ee8c0fe4faeca8293844a8547c6a9a90d093f2526873a19ad4a5e776794c68c742fb834793d2dfcb7fea46c63af4b70fd11cb6e41834e72ee40edb067b292a794990c288d5007e73f349fb383af6a756b8301ad6e5e0aa8cd614399bb3a452376b1575afa6bdaeaafc286cb064bb91edef97c632b6c1113d107fa93a0905098a105043c2f05397f702514439a08a9e5ddc196100721d45c8fc17d2ed659376f8a00bd5cb9a0860e26d8a29d8d6aaf52de97e9346033d6db501a35dbbaf97c20b830cd2d18c2532f3a59cc497ee64c0e57d8d060e5069b28d86edf1adcf59144b221ce3ddaef134b3124fbc7dd000240eff0f5f5f41e83cd7f5bb37c9ae21953fe302b0f6e8b68fa91c6ab99265c64b2fd9cd4942be04321bb5d6d71932376c6f2f88e02422ba6a5e2cb765df93fd5dd0728c6abdaf03bce22e0678a544e2c3636f741b6f4447ee58a8fc656b43ef817932176adbfc2e04b2c812c273cd6cbfa4098f0be036a34221fa02643f5ee2e0b38135f2a18ecd2f16ebc45f8eb31b8ab967a1567ee016904188910861ca1fa205c7adaa194b286893ffe2f4fbe0384c2aef72a4522aeafd3ebc71f9db71eeeef86c48394a1c86d5b36c352cc33a0a2c800bc99e62fd65b3a2fd69e0b53996ec13d8ce483ce9319efd9a85acefabdb5342226febb83fd1daf4b24265f50c61c6de74077ef89b6fecf9f29a1f871af1e9f89b2d345cda7499bd45c42fa5d195a1e1a6ba84851889e730da3b2b916e96152ae0c92154b49719841db7e7cc707ba8a5d7b101eb4ac7b629bb327817910fff61580b59aab78182d1a2e33473d05b00b170b29e331870826cfe45af206aa7d0246bbd8566ca7cfb2d3c10bfa1db7dd48dd786036469ce7282093d78b5e1a5b0fc81a54c8ed4ceac1e5305305e78284ac276f5d7862727aff246e17addde50c670028d572cbfc0be2e4f8b2eb28fa68ad7b4c6c2a239c460441bfb5ea049f23b08563b4e47729a59e5986a61a6093dbd54f8c36ebe87edae01f251cb060ad1364ce677d7e8d5a4a4ca966a7241cc360bc2acb280e5f9e9c1b032ad6a180a35e0c5180b9d16d026c865b252098cc1d99ba7375ca31c7702c0d943d5e3dd2f6861fa55bd46d94b67ed3e52eccd8dd06d968e01897d6de97ed3058d91dd\":hex:\"8e4bc6f8b8c60fe4d68c61d9b159c8693c3151c46749af58da228442d927f23359bd6ccd6c2ec8fa3f00a86cecbfa728e1ad60b821ed22fcd309ba91a4138bc9\"",
    "depends_on:3:0",
    "5",
    "depends_on:3:1",
    "6",
    "depends_on:3:2",
    "7",
};

/* ========================================================================== */
/*                                LOCAL TYPEDEFS                              */
/* ========================================================================== */
static struct
{
    int failed;
    const char *test;
    const char *filename;
    int line_no;
}
test_info;

/* ========================================================================== */
/*                                    MACROS                                  */
/* ========================================================================== */

#define TEST_ASSERT_MBEDTLS( TEST )                 \
    do {                                            \
        if( ! (TEST) )                              \
        {                                           \
            test_fail( #TEST, __LINE__, __FILE__ ); \
            goto exit;                              \
        }                                           \
    } while( 0 )

/* ========================================================================== */
/*                            FUNCTION DECLARATIONS                           */
/* ========================================================================== */

static int dep_check( int dep_id );
static int get_expression( int32_t exp_id, int32_t * out_value );
static int convert_params( size_t cnt , char ** params , int * int_params_store );

int check_test_sha( int func_idx );
int dispatch_test_sha( int func_idx, void ** params );
void test_mbedtls_sha1_wrapper( void ** params );
void test_sha224_wrapper( void ** params );
void test_mbedtls_sha256_wrapper( void ** params );
void test_sha384_wrapper( void ** params );
void test_mbedtls_sha512_wrapper( void ** params );
void test_sha1_selftest_wrapper( void ** params );
void test_sha256_selftest_wrapper( void ** params );
void test_sha512_selftest_wrapper( void ** params );

TestWrapper_t test_funcs_sha[] =
    {
    /* Function Id: 0 */

    #if defined(MBEDTLS_SHA1_C)
        test_mbedtls_sha1_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 1 */

    #if defined(MBEDTLS_SHA256_C)
        test_sha224_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 2 */

    #if defined(MBEDTLS_SHA256_C)
        test_mbedtls_sha256_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 3 */

    #if defined(MBEDTLS_SHA512_C)
        test_sha384_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 4 */

    #if defined(MBEDTLS_SHA512_C)
        test_mbedtls_sha512_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 5 */

    #if defined(MBEDTLS_SHA1_C) && defined(MBEDTLS_SELF_TEST)
        test_sha1_selftest_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 6 */

    #if defined(MBEDTLS_SHA256_C) && defined(MBEDTLS_SELF_TEST)
        test_sha256_selftest_wrapper,
    #else
        NULL,
    #endif
    /* Function Id: 7 */

    #if defined(MBEDTLS_SHA512_C) && defined(MBEDTLS_SELF_TEST)
        test_sha512_selftest_wrapper,
    #else
        NULL,
    #endif

};

/* ========================================================================== */
/*                             FUNCTION DEFINITIONS                           */
/* ========================================================================== */

static void test_fail( const char *test, int line_no, const char* filename )
{
    test_info.failed = 1;
    test_info.test = test;
    test_info.line_no = line_no;
    test_info.filename = filename;
}

#if defined(MBEDTLS_SHA1_C)
void test_mbedtls_sha1( data_t * src_str, data_t * hex_hash_string )
{
    unsigned char output[41] __attribute__ ((aligned (32U)));;

    memset(output, 0x00, 41);

    uint8_t inputBuf[src_str->len] __attribute__ ((aligned (32U)));
    memcpy(&inputBuf[0], src_str->x, src_str->len);

    TEST_ASSERT_MBEDTLS( mbedtls_sha1_ret( inputBuf, src_str->len, output ) == 0 );

    TEST_ASSERT_MBEDTLS( hexcmp( output, hex_hash_string->x, 20, hex_hash_string->len ) == 0 );
exit:
    ;
}

void test_mbedtls_sha1_wrapper( void ** params )
{
    data_t data0 = {(uint8_t *) params[0], *( (uint32_t *) params[1] )};
    data_t data2 = {(uint8_t *) params[2], *( (uint32_t *) params[3] )};

    test_mbedtls_sha1( &data0, &data2 );
}
#endif /* MBEDTLS_SHA1_C */
#if defined(MBEDTLS_SHA256_C)

void test_sha224( data_t * src_str, data_t * hex_hash_string )
{
    unsigned char output[57]  __attribute__ ((aligned (32U)));

    memset(output, 0x00, 57);
    uint8_t inputBuf[src_str->len] __attribute__ ((aligned (32U)));
    memcpy(&inputBuf[0], src_str->x, src_str->len);

    TEST_ASSERT_MBEDTLS( mbedtls_sha256_ret( inputBuf, src_str->len, output, 1 ) == 0 );

    TEST_ASSERT_MBEDTLS( hexcmp( output, hex_hash_string->x, 28, hex_hash_string->len ) == 0 );
exit:
    ;
}

void test_sha224_wrapper( void ** params )
{
    data_t data0 = {(uint8_t *) params[0], *( (uint32_t *) params[1] )};
    data_t data2 = {(uint8_t *) params[2], *( (uint32_t *) params[3] )};

    test_sha224( &data0, &data2 );
}
#endif /* MBEDTLS_SHA256_C */
#if defined(MBEDTLS_SHA256_C)

void test_mbedtls_sha256( data_t * src_str, data_t * hex_hash_string )
{
    unsigned char output[65] __attribute__ ((aligned (32U)));

    memset(output, 0x00, 65);

    uint8_t inputBuf[src_str->len] __attribute__ ((aligned (32U)));
    memcpy(&inputBuf[0], src_str->x, src_str->len);

    TEST_ASSERT_MBEDTLS( mbedtls_sha256_ret( inputBuf, src_str->len, output, 0 ) == 0 );

    TEST_ASSERT_MBEDTLS( hexcmp( output, hex_hash_string->x, 32, hex_hash_string->len ) == 0 );
exit:
    ;
}

void test_mbedtls_sha256_wrapper( void ** params )
{
    data_t data0 = {(uint8_t *) params[0], *( (uint32_t *) params[1] )};
    data_t data2 = {(uint8_t *) params[2], *( (uint32_t *) params[3] )};

    test_mbedtls_sha256( &data0, &data2 );
}
#endif /* MBEDTLS_SHA256_C */
#if defined(MBEDTLS_SHA512_C)

void test_sha384( data_t * src_str, data_t * hex_hash_string )
{
    unsigned char output[97] __attribute__ ((aligned (32U)));

    memset(output, 0x00, 97);
    uint8_t inputBuf[src_str->len] __attribute__ ((aligned (32U)));
    memcpy(&inputBuf[0], src_str->x, src_str->len);

    TEST_ASSERT_MBEDTLS( mbedtls_sha512_ret( inputBuf, src_str->len, output, 1 ) == 0 );

    TEST_ASSERT_MBEDTLS( hexcmp( output, hex_hash_string->x, 48, hex_hash_string->len ) == 0 );
exit:
    ;
}

void test_sha384_wrapper( void ** params )
{
    data_t data0 = {(uint8_t *) params[0], *( (uint32_t *) params[1] )};
    data_t data2 = {(uint8_t *) params[2], *( (uint32_t *) params[3] )};

    test_sha384( &data0, &data2 );
}
#endif /* MBEDTLS_SHA512_C */
#if defined(MBEDTLS_SHA512_C)

void test_mbedtls_sha512( data_t * src_str, data_t * hex_hash_string )
{
    unsigned char output[129] __attribute__ ((aligned (128U)));

    memset(output, 0x00, 129);

    uint8_t inputBuf[src_str->len] __attribute__ ((aligned (128U)));
    memcpy(&inputBuf[0], src_str->x, src_str->len);

    TEST_ASSERT_MBEDTLS( mbedtls_sha512_ret( inputBuf, src_str->len, output, 0 ) == 0 );

    TEST_ASSERT_MBEDTLS( hexcmp( output, hex_hash_string->x, 64, hex_hash_string->len ) == 0 );
exit:
    ;
}

void test_mbedtls_sha512_wrapper( void ** params )
{
    data_t data0 = {(uint8_t *) params[0], *( (uint32_t *) params[1] )};
    data_t data2 = {(uint8_t *) params[2], *( (uint32_t *) params[3] )};

    test_mbedtls_sha512( &data0, &data2 );
}
#endif /* MBEDTLS_SHA512_C */
#if defined(MBEDTLS_SHA1_C)
#if defined(MBEDTLS_SELF_TEST)

void test_sha1_selftest(  )
{
    TEST_ASSERT_MBEDTLS( mbedtls_sha1_self_test( 1 ) == 0 );
exit:
    ;
}

void test_sha1_selftest_wrapper( void ** params )
{
    (void)params;

    test_sha1_selftest(  );
}
#endif /* MBEDTLS_SELF_TEST */
#endif /* MBEDTLS_SHA1_C */
#if defined(MBEDTLS_SHA256_C)
#if defined(MBEDTLS_SELF_TEST)

void test_sha256_selftest(  )
{
    TEST_ASSERT_MBEDTLS( mbedtls_sha256_self_test( 1 ) == 0 );
exit:
    ;
}

void test_sha256_selftest_wrapper( void ** params )
{
    (void)params;

    test_sha256_selftest(  );
}
#endif /* MBEDTLS_SELF_TEST */
#endif /* MBEDTLS_SHA256_C */
#if defined(MBEDTLS_SHA512_C)
#if defined(MBEDTLS_SELF_TEST)

void test_sha512_selftest(  )
{
    TEST_ASSERT_MBEDTLS( mbedtls_sha512_self_test( 1 ) == 0 );
exit:
    ;
}

void test_sha512_selftest_wrapper( void ** params )
{
    (void)params;

    test_sha512_selftest(  );
}
#endif /* MBEDTLS_SELF_TEST */
#endif /* MBEDTLS_SHA512_C */

/**
 * \brief       Checks if the dependency i.e. the compile flag is set.
 *              For optimizing space for embedded targets each dependency
 *              is identified by a unique identifier instead of string literals.
 *              Identifiers and check code is generated by script:
 *              generate_test_code.py
 *
 * \param exp_id    Dependency identifier.
 *
 * \return       DEPENDENCY_SUPPORTED if set else DEPENDENCY_NOT_SUPPORTED
 */
int dep_check( int dep_id )
{
    int ret = DEPENDENCY_NOT_SUPPORTED;

    (void) dep_id;

    switch( dep_id )
    {
        case 0:
        {
        #if defined(MBEDTLS_SHA1_C)
            ret = DEPENDENCY_SUPPORTED;
        #else
            ret = DEPENDENCY_NOT_SUPPORTED;
        #endif
        }
        break;
        case 1:
        {
        #if defined(MBEDTLS_SHA256_C)
                ret = DEPENDENCY_SUPPORTED;
        #else
                ret = DEPENDENCY_NOT_SUPPORTED;
        #endif
        }
        break;
        case 2:
        {
        #if defined(MBEDTLS_SHA512_C)
            ret = DEPENDENCY_SUPPORTED;
        #else
            ret = DEPENDENCY_NOT_SUPPORTED;
        #endif
        }
        break;
        case 3:
        {
        #if defined(MBEDTLS_SELF_TEST)
            ret = DEPENDENCY_SUPPORTED;
        #else
            ret = DEPENDENCY_NOT_SUPPORTED;
        #endif
        }
        break;

        default:
            break;
    }
    return ret;
}

/**
 * \brief       Dispatches test functions based on function index.
 *
 * \param exp_id    Test function index.
 *
 * \return       DISPATCH_TEST_SUCCESS if found
 *               DISPATCH_TEST_FN_NOT_FOUND if not found
 *               DISPATCH_UNSUPPORTED_SUITE if not compile time enabled.
 */
int dispatch_test_sha( int func_idx, void ** params )
{
    int ret = DISPATCH_TEST_SUCCESS;
    TestWrapper_t fp = NULL;

    if ( func_idx < (int)( sizeof( test_funcs_sha ) / sizeof( TestWrapper_t ) ) )
    {
        fp = test_funcs_sha[func_idx];
        if ( fp )
            fp( params );
        else
            ret = DISPATCH_UNSUPPORTED_SUITE;
    }
    else
    {
        ret = DISPATCH_TEST_FN_NOT_FOUND;
    }

    return ret;
}

/**
 * \brief       Checks if test function is supported
 *
 * \param exp_id    Test function index.
 *
 * \return       DISPATCH_TEST_SUCCESS if found
 *               DISPATCH_TEST_FN_NOT_FOUND if not found
 *               DISPATCH_UNSUPPORTED_SUITE if not compile time enabled.
 */
int check_test_sha( int func_idx )
{
    int ret = DISPATCH_TEST_SUCCESS;
    TestWrapper_t fp = NULL;

    if ( func_idx < (int)( sizeof(test_funcs_sha)/sizeof( TestWrapper_t ) ) )
    {
        fp = test_funcs_sha[func_idx];
        if ( fp == NULL )
            ret = DISPATCH_UNSUPPORTED_SUITE;
    }
    else
    {
        ret = DISPATCH_TEST_FN_NOT_FOUND;
    }

    return ret;
}

/**
 * \brief       Converts parameters into test function consumable parameters.
 *              Example: Input:  {"int", "0", "char*", "Hello",
 *                                "hex", "abef", "exp", "1"}
 *                      Output:  {
 *                                0,                // Verified int
 *                                "Hello",          // Verified string
 *                                2, { 0xab, 0xef },// Converted len,hex pair
 *                                9600              // Evaluated expression
 *                               }
 *
 *
 * \param cnt               Parameter array count.
 * \param params            Out array of found parameters.
 * \param int_params_store  Memory for storing processed integer parameters.
 *
 * \return      0 for success else 1
 */
static int convert_params( size_t cnt , char ** params , int * int_params_store )
{
    char ** cur = params;
    char ** out = params;
    int ret = DISPATCH_TEST_SUCCESS;

    while ( cur < params + cnt )
    {
        char * type = *cur++;
        char * val = *cur++;

        if ( strcmp( type, "char*" ) == 0 )
        {
            if ( verify_string( &val ) == 0 )
            {
              *out++ = val;
            }
            else
            {
                ret = ( DISPATCH_INVALID_TEST_DATA );
                break;
            }
        }
        else if ( strcmp( type, "int" ) == 0 )
        {
            if ( verify_int( val, int_params_store ) == 0 )
            {
              *out++ = (char *) int_params_store++;
            }
            else
            {
                ret = ( DISPATCH_INVALID_TEST_DATA );
                break;
            }
        }
        else if ( strcmp( type, "hex" ) == 0 )
        {
            if ( verify_string( &val ) == 0 )
            {
                *int_params_store = unhexify( (unsigned char *) val, val );
                *out++ = val;
                *out++ = (char *)(int_params_store++);
            }
            else
            {
                ret = ( DISPATCH_INVALID_TEST_DATA );
                break;
            }
        }
        else if ( strcmp( type, "exp" ) == 0 )
        {
            int exp_id = strtol( val, NULL, 10 );
            if ( get_expression ( exp_id, int_params_store ) == 0 )
            {
              *out++ = (char *)int_params_store++;
            }
            else
            {
              ret = ( DISPATCH_INVALID_TEST_DATA );
              break;
            }
        }
        else
        {
          ret = ( DISPATCH_INVALID_TEST_DATA );
          break;
        }
    }
    return ret;
}

/**
 * \brief       Evaluates an expression/macro into its literal integer value.
 *              For optimizing space for embedded targets each expression/macro
 *              is identified by a unique identifier instead of string literals.
 *              Identifiers and evaluation code is generated by script:
 *              generate_test_code.py
 *
 * \param exp_id    Expression identifier.
 * \param out_value Pointer to int to hold the integer.
 *
 * \return       0 if exp_id is found. 1 otherwise.
 */
static int get_expression( int32_t exp_id, int32_t * out_value )
{
    int ret = KEY_VALUE_MAPPING_FOUND;

    (void) exp_id;
    (void) out_value;

    switch( exp_id )
    {

        default:
           {
                ret = KEY_VALUE_MAPPING_NOT_FOUND;
           }
           break;
    }
    return ret;
}

int* mbedtls_test_sha()
{
    TEST_MESSAGE("----------------------------TEST FOR SHA----------------------------");
    uint8_t i=0 ,j=0;
    int ret = 0;
    uint8_t function_id = 0;
    char *params[50];
    int int_params[50];
    char* buffer;
    char unity_buffer[100];
    /* Other Local variables */
    int cnt;
    int total_errors = 0, total_tests = 0, total_skipped = 0;
    while ( i < sizeof(data_sha)/sizeof(data_sha[0]) )
    {
        total_tests++;
        buffer = (char*)malloc(sizeof(char)*strlen(data_sha[i]));
        strcpy(buffer, data_sha[i]);
        int unmet_dep_count = 0;
        char *unmet_dependencies[20];
        ret = 0;
        test_info.failed = 0;

        cnt = parse_arguments( buffer, strlen( buffer ), params,
                                    sizeof( params ) / sizeof( params[0] ) );

        if( strcmp( params[0], "depends_on" ) == 0 )
        {
            for( j = 1; j < cnt; j++ )
            {
                int dep_id = strtol( params[j], NULL, 10 );
                if( dep_check( dep_id ) != DEPENDENCY_SUPPORTED )
                {
                    unmet_dependencies[ unmet_dep_count ] = strdup( params[j] );
                    if(  unmet_dependencies[ unmet_dep_count ] == NULL )
                    {
                        TEST_MESSAGE("FATAL: Out of memory" );
                        mbedtls_exit( MBEDTLS_EXIT_FAILURE );
                    }
                    unmet_dep_count++;
                }
            }
            ++i;
            free(buffer);
            buffer = (char*)malloc(sizeof(char)*strlen(data_sha[i]));
            strcpy(buffer, data_sha[i]);
            cnt = parse_arguments( buffer, strlen( buffer ), params,
                                    sizeof( params ) / sizeof( params[0] ) );
        }

        // If there are no unmet dependencies execute the test
        if( unmet_dep_count == 0 )
        {
            test_info.failed = 0;
            function_id = strtol( params[0], NULL, 10 );
            if ( (ret = check_test_sha( function_id )) == DISPATCH_TEST_SUCCESS )
            {
                ret = convert_params( cnt - 1, params + 1, int_params );
                if ( DISPATCH_TEST_SUCCESS == ret )
                {
                    ret = dispatch_test_sha( function_id, (void **)( params + 1 ) );
                }
            }
        }
        if( unmet_dep_count > 0 || ret == DISPATCH_UNSUPPORTED_SUITE )
        {
            total_skipped++;
            sprintf(unity_buffer, "SHA-Test %d : Skipped", total_tests);
            TEST_MESSAGE(unity_buffer);

            if(ret == DISPATCH_UNSUPPORTED_SUITE )
            {
                TEST_MESSAGE("Test Suite not enabled" );
            }

            if(unmet_dep_count > 0 )
            {
                TEST_MESSAGE("Unmet dependencies: " );
            }
            unmet_dep_count = 0;
        }
        else if( ret == DISPATCH_TEST_SUCCESS )
        {
            if( test_info.failed == 0 )
            {
                sprintf(unity_buffer, "SHA-Test %d : PASSED", total_tests);
                TEST_MESSAGE(unity_buffer);
            }
            else
            {
                total_errors++;
                sprintf(unity_buffer, "Test %d : FAILED", total_tests);
                TEST_MESSAGE(unity_buffer);
                sprintf(unity_buffer, "  %s  at line %d, %s",
                                    test_info.test, test_info.line_no,
                                    test_info.filename );
                TEST_MESSAGE(unity_buffer);
            }
        }
        else if( ret == DISPATCH_INVALID_TEST_DATA )
        {
            TEST_MESSAGE("FAILED: FATAL PARSE ERROR" );
        }
        else if( ret == DISPATCH_TEST_FN_NOT_FOUND )
        {
            TEST_MESSAGE("FAILED: FATAL TEST FUNCTION NOT FOUND" );
        }
        else
        {
            total_errors++;
        }
        for( int k = 0; k < unmet_dep_count; k++ )
        {
            free( unmet_dependencies[k] );
        }
        ++i;
        free(buffer);
    }

    if(total_errors == 0 && total_skipped==0)
        TEST_MESSAGE("ALL SHA PASSED" );
    else if(total_errors>0)
        TEST_MESSAGE("SHA TESTS FAILED" );
    else
        TEST_MESSAGE("Some tests skipped" );

    int* test_results = (int*)malloc(3*sizeof(int));
    test_results[0] = total_tests;
    test_results[1] = total_errors;
    test_results[2] = total_skipped;
    return test_results;
}